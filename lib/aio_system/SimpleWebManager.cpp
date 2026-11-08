// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// SimpleWebManager.cpp
// Web server implementation using SimpleHTTPServer to replace AsyncWebServer

#include "SimpleWebManager.h"
#include "ConfigManager.h"
#include "EventLogger.h"
#include "Version.h"
#include "HardwareManager.h"
#include "QNetworkBase.h"
#include "ADProcessor.h"
#include "EncoderProcessor.h"
#include "SimpleOTAHandler.h"
#include "TelemetryWebSocket.h"
#include "AutosteerProcessor.h"
#include "WheelAngleFusion.h"
#include "NAVProcessor.h"
#include "GNSSProcessor.h"
#include "web_pages/CommonStyles.h"  // Common CSS
#include "web_pages/TouchFriendlyEventLoggerPage.h"  // Touch-friendly event logger page
#include "web_pages/TouchFriendlyLogViewerPage.h"  // Touch-friendly log viewer page
#include "web_pages/TouchFriendlyAnalogWorkSwitchPage.h"  // Touch-friendly analog work switch page
#include "web_pages/TouchFriendlyOTAPage.h"  // Touch-friendly OTA update page
#include "web_pages/TouchFriendlyGPSConfigPage.h"  // Touch-friendly GPS configuration page
#include "web_pages/TouchFriendlyHomePage.h"  // Touch-friendly interface
#include "web_pages/TouchFriendlyStyles.h"  // Touch-friendly CSS
#include "web_pages/TouchFriendlyDeviceSettingsPage.h"  // Touch-friendly device settings
#include "web_pages/TouchFriendlyNetworkPage.h"  // Touch-friendly network settings
#include "web_pages/TouchFriendlyAnalogWorkSwitchPage.h"  // Touch-friendly analog work switch
#include "web_pages/TouchFriendlyDragDropCANConfigPage.h"  // Touch-friendly drag-and-drop CAN configuration
#include "web_pages/CANInfoJSON.h"  // CAN info JSON data
#include "web_pages/TouchFriendlyCANConfigUploadPage.h"  // Touch-friendly CAN config upload page
#include "web_pages/TouchFriendlyDNSAliasPage.h"  // DNS alias configuration page
#include "web_pages/TouchFriendlyVWASConfigPage.h"  // Touch-friendly virtual WAS configuration page
#include "web_pages/TouchFriendlyHwMsgPage.h"  // Sprache + Texte der AOG-Hardware-Meldungen (PGN 221)
#include "HeadingMonitor.h"            // HPR heading-health logging (test drive)
#include "HeadingBridge.h"             // heading/roll fall-back diagnostics
#include "CANConfigStorage.h"  // LittleFS storage for custom CAN config
#include <ArduinoJson.h>
#include <QNEthernet.h>
#include "ESP32Interface.h"
#include "UM98xManager.h"
#include "SerialManager.h"

using namespace qindesign::network;

// FPSTR macro for PROGMEM strings
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))

// External references
extern EncoderProcessor* encoderProcessor;
extern GNSSProcessor gnssProcessor;

SimpleWebManager::SimpleWebManager() :
    isRunning(false),
    currentLanguage(WebLanguage::ENGLISH),
    systemReady(false),
    lastModuleAccessTime(0) {
}

SimpleWebManager::~SimpleWebManager() {
    stop();
}

bool FLASHMEM SimpleWebManager::begin(uint16_t port) {
    // Initialize LittleFS for custom configuration storage
    if (!CANConfigStorage::init()) {
        LOG_WARNING(EventSource::NETWORK, "LittleFS init failed - custom CAN config not available");
    }

    // Load language preference from EEPROM
    uint8_t savedLang = EEPROM.read(WEB_CONFIG_ADDR);
    if (savedLang <= 2) {  // 0 = English, 1 = German, 2 = LOCAL (eigene Texte, PGN 221)
        currentLanguage = static_cast<WebLanguage>(savedLang);
    }
    applyLanguageToModules();

    // Setup routes first
    setupRoutes();

    // Register user-configured DNS aliases as owned hostnames so that
    // e.g. "board.aog" serves this board's own web interface (not a module proxy).
    {
        ConfigManager* cfg = ConfigManager::getInstance();
        if (cfg) {
            for (uint8_t i = 0; i < 4; i++) {
                const char* alias = cfg->getDNSAlias(i);
                if (alias && alias[0] != '\0') {
                    httpServer.addOwnedHostname(String(alias));
                }
            }
        }
    }

    // Start HTTP server
    if (!httpServer.begin(port)) {
        LOG_ERROR(EventSource::NETWORK, "Failed to start HTTP server");
        return false;
    }
    
    // Start WebSocket telemetry server on port 8082
    if (!telemetryWS.begin(8082)) {
        LOG_WARNING(EventSource::NETWORK, "Failed to start WebSocket telemetry server");
    }

    // Start Log WebSocket server on port 8083
    if (!logWS.begin(8083)) {
        LOG_WARNING(EventSource::NETWORK, "Failed to start Log WebSocket server");
    } else {
        // Connect LogWebSocket to EventLogger
        EventLogger::getInstance()->setLogWebSocket(&logWS);
    }

    isRunning = true;
    
    IPAddress ip = Ethernet.localIP();
    LOG_INFO(EventSource::NETWORK, "Simple web server started on http://%d.%d.%d.%d:%d", 
             ip[0], ip[1], ip[2], ip[3], port);
    
    return true;
}

void FLASHMEM SimpleWebManager::stop() {
    if (isRunning) {
        // Disconnect LogWebSocket from EventLogger
        EventLogger::getInstance()->setLogWebSocket(nullptr);

        logWS.stop();
        telemetryWS.stop();
        httpServer.stop();
        isRunning = false;
        LOG_INFO(EventSource::NETWORK, "Simple web server stopped");
    }
}

void FLASHMEM SimpleWebManager::handleClient() {
    // Now called by SimpleScheduler at 100Hz
    httpServer.handleClient();
    telemetryWS.handleClients();
    logWS.handleClient();
}


// -----------------------------------------------------------------------
// rewriteModuleHtml
// Rewrites all absolute paths in the HTML of a module page so that
// sub-pages, AJAX calls, and resources are routed correctly through
// the Teensy proxy.
//
// Rewritten patterns:
//   href="/..."           -> href="/ModuleName/..."
//   action="/..."         -> action="/ModuleName/..."
//   src="/..."            -> src="/ModuleName/..."
//   window.open('/'       -> window.open('/ModuleName/'
//   location.href='/      -> location.href='/ModuleName/
//   sendVal('/?...')      -> sendVal('/ModulName/?...')   AJAX-Calls
//   sendVal("/?...")      -> sendVal('/ModulName/?...')
// -----------------------------------------------------------------------
static void rewriteModuleHtml(String& html, const String& modulePrefix) {
    if (modulePrefix.length() == 0) return;

    // Replaces pat (ending with "/") with pat-without-last-slash + modulePrefix + "/"
    // Skips already-rewritten occurrences.
    auto rewritePattern = [&](const char* patCStr) {
        String pat = String(patCStr);
        String skipPat = pat + modulePrefix.substring(1) + "/";
        int searchFrom = 0;
        while (true) {
            int idx = html.indexOf(pat, searchFrom);
            if (idx < 0) break;
            if (html.indexOf(skipPat, idx) == idx) { searchFrom = idx + pat.length(); continue; }
            String replacement = pat.substring(0, pat.length() - 1) + modulePrefix + "/";
            html = html.substring(0, idx) + replacement + html.substring(idx + pat.length());
            searchFrom = idx + replacement.length();
        }
    };

    rewritePattern("href=\"/");
    rewritePattern("href='/");
    rewritePattern("action=\"/");
    rewritePattern("action='/");
    rewritePattern("src=\"/");
    rewritePattern("src='/");
    rewritePattern("window.open('/");
    rewritePattern("window.open(\"/");
    rewritePattern("location.href='/");
    rewritePattern("location.href=\"/");

    // sendVal AJAX calls: sendVal('/?x') -> sendVal('/ModuleName/?x')
    auto rewriteSendVal = [&](const char* openQuote) {
        String pat = String(openQuote) + "/";
        String skipPat = String(openQuote) + modulePrefix + "/";
        int searchFrom = 0;
        while (true) {
            int idx = html.indexOf(pat, searchFrom);
            if (idx < 0) break;
            if (html.indexOf(skipPat, idx) == idx) { searchFrom = idx + pat.length(); continue; }
            // Avoid double /
            if (idx + (int)pat.length() < (int)html.length() &&
                html.charAt(idx + pat.length()) == '/') { searchFrom = idx + pat.length(); continue; }
            String replacement = String(openQuote) + modulePrefix + "/";
            html = html.substring(0, idx) + replacement + html.substring(idx + pat.length());
            searchFrom = idx + replacement.length();
        }
    };

    rewriteSendVal("sendVal('");
    rewriteSendVal("sendVal(\"");

    // fetch() calls: fetch('/...') -> fetch('/ModuleName/...')
    auto rewriteFetch = [&](const char* openQuote) {
        String pat = String("fetch(") + openQuote + "/";
        String skipPat = String("fetch(") + openQuote + modulePrefix.substring(1) + "/";
        int searchFrom = 0;
        while (true) {
            int idx = html.indexOf(pat, searchFrom);
            if (idx < 0) break;
            if (html.indexOf(skipPat, idx) == idx) { searchFrom = idx + pat.length(); continue; }
            // Avoid double /
            if (idx + (int)pat.length() < (int)html.length() &&
                html.charAt(idx + pat.length()) == '/') { searchFrom = idx + pat.length(); continue; }
            String replacement = String("fetch(") + openQuote + modulePrefix + "/";
            html = html.substring(0, idx) + replacement + html.substring(idx + pat.length());
            searchFrom = idx + replacement.length();
        }
    };
    rewriteFetch("'");
    rewriteFetch("\"");
}

// RAM1-Entlastung 2026-10-29: laeuft EINMAL beim Boot (Routen-Registrierung), muss also
// nicht im ITCM liegen. Ohne FLASHMEM belegte die Funktion 3.3 KB von den 512 KB, die sich
// ITCM (Code) und DTCM (Variablen) teilen - der Link scheiterte mit -928 Byte.
// ---------------------------------------------------------------------------
// Sprache: EIN Weg fuer Web-UI und AOG-Hardware-Meldungen (2026-11-06)
// ---------------------------------------------------------------------------
// Betreiber-Entscheidung: die Meldungssprache ist DIESELBE wie die des Web-UI (kein
// eigenes EEPROM-Byte). EN und DE sind im Flash kompiliert, LOCAL kommt aus dem
// EEPROM-Block 1300 (aog_hwmsg_store.h). Die Zahlenwerte sind absichtlich gleich
// (0/1/2), damit die Zuordnung eine reine Umwandlung ist.
void SimpleWebManager::setLanguage(WebLanguage lang) {
    currentLanguage = lang;
    EEPROM.write(WEB_CONFIG_ADDR, static_cast<uint8_t>(lang));
    applyLanguageToModules();
}

void SimpleWebManager::applyLanguageToModules() {
    // PGN 221: Hardware-Meldungen nach AgOpenGPS. getInstance() legt das Objekt an,
    // falls es noch nicht existiert - hier ist also kein Null-Fall zu behandeln.
    if (AutosteerProcessor* a = AutosteerProcessor::getInstance()) {
        a->setHardwareMessageLanguage(static_cast<AogMsgLang>(currentLanguage));
    }
}

// 2026-11-06: Ziel nach einem Sprachwechsel. Es wird nur ein fester Seitenname
// ausgewertet (keine freie URL - eine offene Umleitung waere ein Einfallstor).
// Ohne Parameter bleibt es bei der Startseite, wie bisher.
static const char* langRedirectTarget(const String& query) {
    if (query.indexOf("back=hwmsg") >= 0) { return "/hwmsg"; }
    return "/";
}

void FLASHMEM SimpleWebManager::setupRoutes() {
    // Home page - now using touch-friendly interface
    httpServer.on("/", [this](EthernetClient& client, const String& method, const String& query) {
        sendTouchHomePage(client);
    });
    
    // CSS for touch-friendly interface
    httpServer.on("/touch.css", [this](EthernetClient& client, const String& method, const String& query) {
        extern const char TOUCH_FRIENDLY_CSS[];
        SimpleHTTPServer::send(client, 200, "text/css", FPSTR(TOUCH_FRIENDLY_CSS));
    });
    
    // favicon.ico - return 204 immediately, never call the proxy.
    // Without this route every browser request for favicon.ico would land in
    // the module proxy and trigger a 15 s timeout that hangs the browser.
    httpServer.on("/favicon.ico", [](EthernetClient& client, const String& method, const String& query) {
        client.print("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n");
        client.flush();
    });

    // API status endpoint
    httpServer.on("/api/status", [this](EthernetClient& client, const String& method, const String& query) {
        handleApiStatus(client);
    });

    // /api/modules -> ESP32 module list (DynModules via erw. PGN 203) via UART proxy
    // Returns the modules[] array from the ESP32 buildStatusJson()
    httpServer.on("/api/modules", [this](EthernetClient& client, const String& method, const String& query) {
        handleApiModules(client);
    });
    
    // EventLogger page
    httpServer.on("/eventlogger", [this](EthernetClient& client, const String& method, const String& query) {
        sendEventLoggerPage(client);
    });

    // Log viewer page (tablet-friendly)
    httpServer.on("/logs", [this](EthernetClient& client, const String& method, const String& query) {
        sendLogViewerPage(client);
    });

    // Network settings page
    httpServer.on("/network", [this](EthernetClient& client, const String& method, const String& query) {
        sendNetworkPage(client);
    });
    
    // OTA Update page
    httpServer.on("/ota", [this](EthernetClient& client, const String& method, const String& query) {
        sendOTAPage(client);
    });
    
    // Device Settings page
    httpServer.on("/device", [this](EthernetClient& client, const String& method, const String& query) {
        sendDeviceSettingsPage(client);
    });
    
    // Analog Work Switch page
    httpServer.on("/analogworkswitch", [this](EthernetClient& client, const String& method, const String& query) {
        sendAnalogWorkSwitchPage(client);
    });

    // CAN Configuration page
    httpServer.on("/can", [this](EthernetClient& client, const String& method, const String& query) {
        sendCANConfigPage(client);
    });

    // CAN Config Upload page
    httpServer.on("/can/upload", [this](EthernetClient& client, const String& method, const String& query) {
        sendCANConfigUploadPage(client);
    });

    // WAS Demo page removed - using WebSocket telemetry instead
    
    // Language selection
    // Ziel nach dem Umschalten: die Seite, die den Wechsel angefordert hat. Vorher stand hier
    // fest "/" - der Bediener landete auf der Startseite und musste zurueckklicken (gemeldet
    // 2026-11-06). Es wird nur ein fester Name ausgewertet, keine freie URL.
    httpServer.on("/lang/en", [this](EthernetClient& client, const String& method, const String& query) {
        setLanguage(WebLanguage::ENGLISH);
        SimpleHTTPServer::redirect(client, langRedirectTarget(query));
    });
    
    httpServer.on("/lang/de", [this](EthernetClient& client, const String& method, const String& query) {
        setLanguage(WebLanguage::GERMAN);
        SimpleHTTPServer::redirect(client, langRedirectTarget(query));
    });

    // LOCAL (2026-11-06): dritte Sprache - die Texte der AOG-Hardware-Meldungen (PGN 221)
    // tippt der Bediener selbst ein (EEPROM-Block 1300). Die Web-Oberflaeche bleibt englisch,
    // weil alle Texte dort auf "== GERMAN ? ... : ..." pruefen.
    httpServer.on("/lang/local", [this](EthernetClient& client, const String& method, const String& query) {
        setLanguage(WebLanguage::LOCAL);
        SimpleHTTPServer::redirect(client, langRedirectTarget(query));
    });

    // Sprache + Texte der AOG-Hardware-Meldungen (PGN 221)
    httpServer.on("/hwmsg", [this](EthernetClient& client, const String& method, const String& query) {
        sendHwMsgPage(client);
    });
    httpServer.on("/api/hwmsg/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleHwMsgConfig(client, method);
    });
    
    // API endpoints
    httpServer.on("/api/restart", [](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"restarting\"}");
            delay(100);
            SCB_AIRCR = 0x05FA0004;  // System reset
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });
    
    // EventLogger API
    httpServer.on("/api/eventlogger/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleEventLoggerConfig(client, method);
    });

    // Log viewer API
    httpServer.on("/api/logs/data", [this](EthernetClient& client, const String& method, const String& query) {
        handleLogViewerData(client);
    });

    // Network API
    httpServer.on("/api/network/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleNetworkConfig(client, method);
    });
    
    // Device settings API
    httpServer.on("/api/device/settings", [this](EthernetClient& client, const String& method, const String& query) {
        handleDeviceSettings(client, method);
    });
    
    // Analog work switch API
    httpServer.on("/api/analogworkswitch/status", [this](EthernetClient& client, const String& method, const String& query) {
        handleAnalogWorkSwitchStatus(client);
    });
    
    httpServer.on("/api/analogworkswitch/config", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleAnalogWorkSwitchConfig(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });
    
    httpServer.on("/api/analogworkswitch/setpoint", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleAnalogWorkSwitchSetpoint(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });

    // CAN configuration API
    httpServer.on("/api/can/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleCANConfig(client, method);
    });

    // CAN info JSON API (brand capabilities, functions, etc.)
    httpServer.on("/api/can/info", [this](EthernetClient& client, const String& method, const String& query) {
        handleCANInfo(client);
    });

    // CAN config upload endpoint
    httpServer.on("/api/can/config/upload", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleCANConfigUpload(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });

    // CAN config restore default endpoint
    httpServer.on("/api/can/config/restore", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleCANConfigRestore(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });

    // CAN config status endpoint
    httpServer.on("/api/can/config/status", [this](EthernetClient& client, const String& method, const String& query) {
        handleCANConfigStatus(client);
    });

    // OTA upload endpoint
    httpServer.on("/api/ota/upload", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleOTAUpload(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });
    
    // UM98x GPS configuration page
    httpServer.on("/um98x-config", [this](EthernetClient& client, const String& method, const String& query) {
        sendUM98xConfigPage(client);
    });
    
    // GPS config shortcut
    httpServer.on("/gps", [this](EthernetClient& client, const String& method, const String& query) {
        sendUM98xConfigPage(client);
    });

    // DNS alias configuration page
    httpServer.on("/dns-alias", [this](EthernetClient& client, const String& method, const String& query) {
        sendDNSAliasPage(client);
    });

    // Virtual WAS configuration page
    httpServer.on("/vwas", [this](EthernetClient& client, const String& method, const String& query) {
        sendVWASConfigPage(client);
    });
    httpServer.on("/api/vwas/status", [this](EthernetClient& client, const String& method, const String& query) {
        handleVWASStatus(client);
    });
    httpServer.on("/api/vwas/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleVWASConfig(client, method);
    });
    // Antenna geometry of the dual-antenna baseline (HeadingBridge): the mechanical spacing
    // and the dual-antenna heading offset. Own endpoint: both are heading settings, and
    // /api/vwas/config answers 503 without an active VWAS.
    httpServer.on("/api/bridge/geometry", [this](EthernetClient& client, const String& method, const String& query) {
        handleBridgeGeometry(client, method);
    });

    // Triggers a CSV dump of the HPR heading-health ring buffer onto the USB
    // serial console. Needed because the automatic dump only fires when a
    // heading-loss event COMPLETES - if the receiver never degrades during a
    // test drive, there would otherwise be no baseline data at all.
    httpServer.on("/api/heading/dump", [this](EthernetClient& client, const String& method, const String& query) {
        handleHeadingDump(client);
    });

    // DNS alias configuration API
    httpServer.on("/api/dns-alias/config", [this](EthernetClient& client, const String& method, const String& query) {
        handleDNSAliasConfig(client, method);
    });
    
    // UM98x API endpoints
    httpServer.on("/api/um98x/read", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "GET") {
            handleUM98xRead(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });
    
    httpServer.on("/api/um98x/write", [this](EthernetClient& client, const String& method, const String& query) {
        if (method == "POST") {
            handleUM98xWrite(client);
        } else {
            SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
        }
    });
    
    // Note: Removed polling endpoints like /api/was/angle and /api/encoder/count
    // These are now provided via WebSocket telemetry

    // -----------------------------------------------------------------------
    // /wifi/ -> ESP32 web interface (original make_HTML01 page of the ESP32)
    // Forwarded via UART proxy with HTML rewriting for absolute paths.
    // No pre-scan: the ESP32 scans itself every 2.5 s; the scan button
    // on the page can be used manually.
    // -----------------------------------------------------------------------
    httpServer.on("/wifi/", [this](EthernetClient& client, const String& method, const String& query) {
        // ESP32 not connected: return error immediately, no proxy timeout
        if (!esp32Interface.isDetected()) {
            SimpleHTTPServer::send(client, 503, "text/html; charset=utf-8",
                "<h2>503 - ESP32 not connected</h2>"
                "<p>The ESP32 (AiO WiFi Bridge) is not responding. "
                "Please check wiring and firmware.</p>"
                "<p><a href='/'>Back</a></p>");
            return;
        }

        // Forward query parameters to the ESP32 (e.g. ?ACTION=3 for the scan button)
        String targetUrl = "192.168.137.1/";
        if (query.length() > 0) targetUrl += "?" + query;

        // Send chunked headers immediately so the browser does not time out
        client.print("HTTP/1.1 200 OK\r\n");
        client.print("Content-Type: text/html; charset=utf-8\r\n");
        client.print("Transfer-Encoding: chunked\r\n");
        client.print("Connection: close\r\n");
        client.print("\r\n");
        client.flush();

        // Load original ESP32 home page (make_HTML01)
        String responseBody;
        bool ok = esp32Interface.proxyRequest(targetUrl, responseBody);

        if (!ok || responseBody.length() == 0) {
            // Send error as chunk
            const char* errBody =
                "<h2>502 - ESP32 unreachable</h2>"
                "<p>The ESP32 is not responding to UART requests.</p>";
            client.printf("%X\r\n", (unsigned int)strlen(errBody));
            client.print(errBody);
            client.print("\r\n");
            client.print("0\r\n\r\n");
            client.flush();
            return;
        }

        // HTML rewriting for the ESP32 bridge page:
        // Only rewrite sendVal AJAX calls and plain navigation links,
        // NOT the module buttons (window.open('/sc/') etc.) - those should
        // run directly through the normal module proxy (/sc/ -> ESP32 -> 192.168.137.71).
        //
        // Specifically: sendVal('/?x') -> sendVal('/wifi/?x')
        //               href="/settings" -> href="/wifi/settings"
        //               window.open('/serverIndex') -> window.open('/wifi/serverIndex')
        // NOT: window.open('/sc/') - stays as /sc/ (module proxy)
        {
            // Rewrite sendVal calls
            // sendVal('/?  pattern
            String pat1 = "sendVal('/?"; 
            String rep1 = "sendVal('/wifi/?"; 
            int pos = 0;
            while ((pos = responseBody.indexOf(pat1, pos)) >= 0) {
                responseBody = responseBody.substring(0, pos) + rep1 +
                               responseBody.substring(pos + pat1.length());
                pos += rep1.length();
            }
            // sendValEnc: builds fetch('/?key=val') -> must become '/wifi/?key=val'
            // sendValEnc('key',val) -> sendVal('/?key='+encodeURIComponent(val))
            // So the sendVal rewrite above is sufficient - sendValEnc calls sendVal.
            // Only needed when sendValEnc uses the already-rewritten sendVal.
            // Since sendValEnc appears as a string literal in the HTML and calls sendVal,
            // the sendVal rewrite covers it: sendVal('/?x') -> sendVal('/wifi/?x').
            // Additionally: guard direct fetch() calls inside sendValEnc:
            responseBody.replace("sendValEnc(", "sendValEncWiFi(");
            // Insert new helper function in the same script block
            String encFunc = "<script>function sendValEncWiFi(k,v)"
                             "{sendVal('/wifi/?'+ k +'='+encodeURIComponent(v));}</script>";
            int bodyEnd = responseBody.lastIndexOf("</body>");
            if (bodyEnd >= 0) responseBody = responseBody.substring(0, bodyEnd)
                                            + encFunc + responseBody.substring(bodyEnd);
            else responseBody += encFunc;
            // Rewrite href and window.open for known ESP32 routes
            static const char* const ESP32_ROUTES[] = {
                "/settings", "/serverIndex", "/gpio", nullptr
            };
            for (int ri = 0; ESP32_ROUTES[ri] != nullptr; ri++) {
                String r = String(ESP32_ROUTES[ri]);
                // href exakt
                responseBody.replace("href=\"" + r + "\"", "href=\"/wifi" + r + "\"");
                responseBody.replace("href='" + r + "'", "href='/wifi" + r + "'");
                // window.open: also with query string (?lang=... etc.)
                responseBody.replace("window.open('" + r + "'" , "window.open('/wifi" + r + "'");
                responseBody.replace("window.open('" + r + "?" , "window.open('/wifi" + r + "?");
                responseBody.replace("window.open(\"" + r + "\"", "window.open(\"/wifi" + r + "\"");
                responseBody.replace("window.open(\"" + r + "?", "window.open(\"/wifi" + r + "?");
            }
        }

        // Send body as chunk
        size_t bodyLen = responseBody.length();
        client.printf("%X\r\n", (unsigned int)bodyLen);
        const size_t CHUNK = 1460;
        size_t sent = 0;
        const char* bodyData = responseBody.c_str();
        uint32_t deadline = millis() + 15000;
        while (sent < bodyLen && millis() < deadline) {
            size_t toSend = bodyLen - sent;
            if (toSend > CHUNK) toSend = CHUNK;
            size_t written = client.write(
                reinterpret_cast<const uint8_t*>(bodyData + sent), toSend);
            if (written > 0) { sent += written; }
            else { Ethernet.loop(); delayMicroseconds(100); }
        }
        client.print("\r\n");
        client.print("0\r\n\r\n");
        client.flush();
    });

    // -----------------------------------------------------------------------
    // Catch-all: all unrecognised paths are handled as module proxy.
    // Example: http://192.168.5.126/SectionControl/ -> ESP32 -> 192.168.137.71/
    //          http://192.168.5.126/sc/             -> ESP32 -> 192.168.137.71/
    // The ESP32 web server knows the module names and their IPs and forwards.
    // -----------------------------------------------------------------------
    httpServer.onNotFound([this](EthernetClient& client, const String& path, const String& query) {
        handleModuleProxy(client, path, query);
    });

    LOG_INFO(EventSource::NETWORK, "Simple web routes configured");
}


// -----------------------------------------------------------------------
// handleModuleProxy
// Forwards requests to WiFi modules via ESP32 UART.
// Routing: module prefix in path -> set sticky; without prefix but
// sticky active -> assign sub-request to the module; else passthrough.
// HTML rewriting is applied to all HTML responses from modules.
// -----------------------------------------------------------------------
void FLASHMEM SimpleWebManager::handleModuleProxy(EthernetClient& client, const String& path, const String& query) {
    static constexpr uint32_t STICKY_MODULE_TIMEOUT_MS = 30000;
    static const char ESP32_AP_IP[] = "192.168.137.1";

    // No ESP32 connected: skip blocking proxy call.
    // Returns a 503 response immediately instead of waiting 15 s.
    if (!esp32Interface.isDetected()) {
        SimpleHTTPServer::send(client, 503, "text/html; charset=utf-8",
            String("<h2>503 - ESP32 not connected</h2>")
            + "<p>Path: " + path + "</p>"
            + "<p>The ESP32 (AiO WiFi Bridge) is not responding.</p>"
            + "<p><a href='/'>Back</a></p>");
        lastActiveModuleName = "";
        lastActiveModulePrefix = "";
        return;
    }

    String normPath = path;
    if (!normPath.startsWith("/")) normPath = "/" + normPath;
    String queryStr = query.length() > 0 ? "?" + query : "";

    String firstSeg = normPath.substring(1);
    int slashPos = firstSeg.indexOf('/');
    if (slashPos >= 0) firstSeg = firstSeg.substring(0, slashPos);
    bool firstSegLooksLikeModule = (firstSeg.length() > 0) && (firstSeg.indexOf('.') < 0);

    // Detect IP address as first segment (e.g. /192.168.137.1 or /192.168.137.71/settings).
    // IP-path requests must bypass sticky session completely.
    bool firstSegIsIP = false;
    if (!firstSegLooksLikeModule && firstSeg.indexOf('.') >= 0) {
        firstSegIsIP = true;
        for (int i = 0; i < (int)firstSeg.length(); i++) {
            char c = firstSeg[i];
            if (!(c == '.' || (c >= '0' && c <= '9'))) { firstSegIsIP = false; break; }
        }
    }

    String forwardUrl;
    bool doRewrite = false;

    if (firstSegIsIP) {
        // Direct IP-path: /192.168.137.x/... -> send to Bridge for forwarding.
        // Bridge self-detects its own IP and serves local pages; other IPs are TCP-proxied.
        // Never use or update the sticky session for IP-path requests.
        forwardUrl = String(ESP32_AP_IP) + normPath + queryStr;
        doRewrite = false;
        LOG_INFO(EventSource::NETWORK, "Module proxy [IP path]: %s -> http://%s",
                 path.c_str(), forwardUrl.c_str());

    } else if (firstSegLooksLikeModule) {
        lastActiveModuleName   = firstSeg;
        lastActiveModulePrefix = "/" + firstSeg;
        lastModuleAccessTime   = millis();

        // Special case: /wifi/xxx -> bridge page directly without prefix
        // /wifi/ route matches only exactly /wifi/; /wifi/settings etc.
        // land here. The bridge knows /settings, not /wifi/settings.
        if (firstSeg == "wifi") {
            // Extract sub-path: /wifi/settings -> /settings
            // "/wifi" = 5 chars, so from index 5 = "/settings"
            String subPath = normPath.substring(5); // /wifi/settings -> /settings
            if (subPath.length() == 0 || subPath[0] != '/') subPath = "/";
            forwardUrl = String(ESP32_AP_IP) + subPath + queryStr;
            doRewrite  = true;
            LOG_INFO(EventSource::NETWORK, "ESP32 proxy [subpath]: %s -> http://%s",
                     path.c_str(), forwardUrl.c_str());
        } else {
            forwardUrl = String(ESP32_AP_IP) + normPath + queryStr;
            doRewrite  = true;
            LOG_INFO(EventSource::NETWORK, "Module proxy [%s]: %s -> http://%s",
                     firstSeg.c_str(), path.c_str(), forwardUrl.c_str());
        }

    } else if (lastActiveModuleName.length() > 0 &&
               (millis() - lastModuleAccessTime) < STICKY_MODULE_TIMEOUT_MS) {
        forwardUrl = String(ESP32_AP_IP) + lastActiveModulePrefix + normPath + queryStr;
        lastModuleAccessTime = millis();
        doRewrite  = true;
        LOG_INFO(EventSource::NETWORK, "Module proxy [%s sticky]: %s -> http://%s",
                 lastActiveModuleName.c_str(), path.c_str(), forwardUrl.c_str());

    } else {
        if (lastActiveModuleName.length() > 0) { lastActiveModuleName = ""; lastActiveModulePrefix = ""; }
        forwardUrl = String(ESP32_AP_IP) + normPath + queryStr;
        LOG_INFO(EventSource::NETWORK, "Module proxy [no sticky]: %s -> http://%s",
                 path.c_str(), forwardUrl.c_str());
    }

    // Content-Type
    String contentType = "text/html; charset=utf-8";
    if      (path.endsWith(".css"))   contentType = "text/css";
    else if (path.endsWith(".js"))    contentType = "application/javascript";
    else if (path.endsWith(".ico"))   contentType = "image/x-icon";
    else if (path.endsWith(".png"))   contentType = "image/png";
    else if (path.endsWith(".jpg"))   contentType = "image/jpeg";
    else if (path.endsWith(".gif"))   contentType = "image/gif";
    else if (path.endsWith(".json"))  contentType = "application/json";
    else if (path.endsWith(".xml"))   contentType = "text/xml";
    else if (path.endsWith(".svg"))   contentType = "image/svg+xml";

    // Send headers immediately (prevents browser timeout during UART proxy)
    client.print("HTTP/1.1 200 OK\r\n");
    client.printf("Content-Type: %s\r\n", contentType.c_str());
    client.print("Transfer-Encoding: chunked\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.flush();

    // UART-Proxy ausfuehren
    String responseBody;
    bool ok = esp32Interface.proxyRequest(forwardUrl, responseBody);

    if (!ok) {
        const char* errBody = "<h2>502 - Module unreachable</h2>";
        client.printf("%X\r\n", (unsigned int)strlen(errBody));
        client.print(errBody);
        client.print("\r\n");
    } else {
        if (doRewrite && contentType == "text/html; charset=utf-8" && lastActiveModulePrefix.length() > 0) {
            rewriteModuleHtml(responseBody, lastActiveModulePrefix);
        }
        size_t bodyLen = responseBody.length();
        client.printf("%X\r\n", (unsigned int)bodyLen);
        const size_t CHUNK = 1460;
        size_t sent = 0;
        const char* data = responseBody.c_str();
        uint32_t deadline = millis() + 15000;
        while (sent < bodyLen && millis() < deadline) {
            size_t toSend = bodyLen - sent;
            if (toSend > CHUNK) toSend = CHUNK;
            size_t written = client.write(reinterpret_cast<const uint8_t*>(data + sent), toSend);
            if (written > 0) { sent += written; }
            else { Ethernet.loop(); delayMicroseconds(100); }
        }
        client.print("\r\n");
    }
    client.print("0\r\n\r\n");
    client.flush();
}

// Page handlers

void SimpleWebManager::sendHomePage(EthernetClient& client) {
    extern const char SIMPLE_HOME_PAGE[];
    String html = FPSTR(SIMPLE_HOME_PAGE);
    html.replace("%CSS_STYLES%", FPSTR(COMMON_CSS));
    html.replace("%FIRMWARE_VERSION%", FIRMWARE_VERSION);
    
    SimpleHTTPServer::send(client, 200, "text/html; charset=utf-8", html);
}

void SimpleWebManager::sendTouchHomePage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_HOME_PAGE[];
    
    // Send directly from PROGMEM without string manipulation
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_HOME_PAGE);
}

void SimpleWebManager::sendEventLoggerPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_EVENT_LOGGER_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_EVENT_LOGGER_PAGE);
}

void SimpleWebManager::sendLogViewerPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_LOG_VIEWER_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_LOG_VIEWER_PAGE);
}

void SimpleWebManager::sendNetworkPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_NETWORK_PAGE[];
    
    // Send directly from PROGMEM without string manipulation
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_NETWORK_PAGE);
}

void SimpleWebManager::sendOTAPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_OTA_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_OTA_PAGE);
}

void SimpleWebManager::sendDeviceSettingsPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_DEVICE_SETTINGS_PAGE[];
    
    // Send directly from PROGMEM without any string manipulation
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_DEVICE_SETTINGS_PAGE);
}

void SimpleWebManager::sendAnalogWorkSwitchPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_ANALOG_WORK_SWITCH_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_ANALOG_WORK_SWITCH_PAGE);
}

void SimpleWebManager::sendCANConfigPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_DRAG_DROP_CAN_CONFIG_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_DRAG_DROP_CAN_CONFIG_PAGE);
}

void SimpleWebManager::sendCANConfigUploadPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_CAN_CONFIG_UPLOAD_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_CAN_CONFIG_UPLOAD_PAGE);
}

void SimpleWebManager::sendVWASConfigPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_VWAS_CONFIG_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_VWAS_CONFIG_PAGE);
}

// ---------------------------------------------------------------------------
// Sprache + Texte der AOG-Hardware-Meldungen (PGN 221) - 2026-11-06
// ---------------------------------------------------------------------------
void SimpleWebManager::sendHwMsgPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_HW_MSG_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_HW_MSG_PAGE);
}

void FLASHMEM SimpleWebManager::handleHwMsgConfig(EthernetClient& client, const String& method) {
    AutosteerProcessor* a = AutosteerProcessor::getInstance();
    if (a == nullptr) {
        SimpleHTTPServer::send(client, 503, "application/json",
                                "{\"error\":\"Autosteer not active\"}");
        return;
    }

    if (method == "POST") {
        const String body = readPostBody(client);
        JsonDocument doc;
        if (deserializeJson(doc, body) != DeserializationError::Ok) {
            SimpleHTTPServer::sendJSON(client,
                "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }
        // Ein Feld, das NICHT im JSON steht, bleibt unveraendert; ein LEERES loescht den
        // Text (dann gilt Englisch). Die Grenze (23 Zeichen) setzt der Sender ohnehin
        // durch (AOG_LOCAL_LEN), das Web-Formular begrenzt zusaetzlich per maxlength.
        if (!doc["weakFix"].isNull())  { a->setLocalHardwareMessageText(AogMsgId::WeakFix,  doc["weakFix"]  | ""); }
        if (!doc["bridge"].isNull())   { a->setLocalHardwareMessageText(AogMsgId::Bridge,   doc["bridge"]   | ""); }
        if (!doc["noCourse"].isNull()) { a->setLocalHardwareMessageText(AogMsgId::NoCourse, doc["noCourse"] | ""); }
        if (!doc["posJump"].isNull())  { a->setLocalHardwareMessageText(AogMsgId::PosJump,  doc["posJump"]  | ""); }
        // Hauptschalter (kommt vom Slider als erstes Element der Seite). Er darf auch
        // allein kommen - dann bleibt an den Texten alles, wie es ist.
        if (!doc["enabled"].isNull())  { a->setHardwareMessagesEnabled(doc["enabled"].as<bool>()); }
        // Nur hier wird das EEPROM geschrieben (Block 2000, Hauptschalter + Texte zusammen).
        a->saveLocalHardwareMessageTexts();
        LOG_INFO(EventSource::NETWORK, "PGN221: lokale Meldungstexte gespeichert (aktiv=%d)",
                 a->hardwareMessagesEnabled() ? 1 : 0);
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"ok\"}");
        return;
    }

    // GET: die lokalen Texte, die AKTIVEN Texte und die aktive Sprache.
    // Die aktiven Texte sind wichtig (Betreiber-Befund 2026-11-06): nach dem Umschalten auf
    // Deutsch blieben die Eingabefelder leer, weil sie die LOCAL-Texte zeigen - der Bediener
    // sah nicht, dass sehr wohl deutsche Texte gesendet werden. "active" zeigt genau das,
    // was jetzt nach AgOpenGPS geht (Rueckfallkette inklusive).
    JsonDocument doc;
    doc["weakFix"]  = a->getLocalHardwareMessageText(AogMsgId::WeakFix);
    doc["bridge"]   = a->getLocalHardwareMessageText(AogMsgId::Bridge);
    doc["noCourse"] = a->getLocalHardwareMessageText(AogMsgId::NoCourse);
    doc["posJump"]  = a->getLocalHardwareMessageText(AogMsgId::PosJump);
    doc["lang"]     = static_cast<uint8_t>(currentLanguage);
    doc["enabled"]  = a->hardwareMessagesEnabled();
    JsonObject act  = doc["active"].to<JsonObject>();
    act["weakFix"]  = a->getHardwareMessageText(AogMsgId::WeakFix);
    act["bridge"]   = a->getHardwareMessageText(AogMsgId::Bridge);
    act["noCourse"] = a->getHardwareMessageText(AogMsgId::NoCourse);
    act["posJump"]  = a->getHardwareMessageText(AogMsgId::PosJump);
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

// WAS Demo page removed - using WebSocket telemetry instead

// API handlers

void FLASHMEM SimpleWebManager::handleApiStatus(EthernetClient& client) {
    StaticJsonDocument<512> doc;
    
    // Basic system info
    doc["version"] = FIRMWARE_VERSION;
    doc["uptime"] = millis();
    // Free memory calculation for Teensy 4.1
    extern unsigned long _heap_end;
    extern char *__brkval;
    int freeMemory = (char *)&_heap_end - __brkval;
    doc["freeMemory"] = freeMemory;
    
    // Network info
    JsonObject network = doc.createNestedObject("network");
    IPAddress localIP = Ethernet.localIP();
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);
    network["ip"] = ipStr;
    network["connected"] = Ethernet.linkState();
    network["linkSpeed"] = Ethernet.linkSpeed();
    
    // Module info
    doc["deviceType"] = "Steer";  // Fixed for steer module
    doc["moduleId"] = 126;  // Fixed steer module ID
    
    // ESP32 status
    doc["esp32Detected"] = esp32Interface.isDetected();
    doc["esp32Active"] = esp32Interface.isDetected();  // Active if detected
    
    // System status
    doc["systemHealthy"] = true;
    
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

// -----------------------------------------------------------------------
// handleApiModules
// Fetches the module list from the ESP32 via UART proxy (/api/status)
// and returns it as JSON to the browser.
// Format: { "wifiOnline": true, "modules": [...] }
// Each module: { "name", "path", "ip", "desc", "online", "src" }
// -----------------------------------------------------------------------
void FLASHMEM SimpleWebManager::handleApiModules(EthernetClient& client) {
    // Return early if the ESP32 is not detected.
    // proxyRequest() would otherwise block for 15 s and hang the browser.
    if (!esp32Interface.isDetected()) {
        SimpleHTTPServer::sendJSON(client, "{\"wifiOnline\":false,\"modules\":[]}");
        return;
    }

    String responseBody;
    bool esp32ok = esp32Interface.proxyRequest("192.168.137.1/api/status", responseBody);

    if (!esp32ok || responseBody.length() == 0) {
        SimpleHTTPServer::sendJSON(client, "{\"wifiOnline\":false,\"modules\":[]}");
        return;
    }

    // Extract bridge metadata from the ESP32 response
    auto extractStr = [&](const String& key) -> String {
        String pat = "\"" + key + "\":\"";
        int pos = responseBody.indexOf(pat);
        if (pos < 0) return "";
        pos += pat.length();
        int end = responseBody.indexOf('"', pos);
        if (end < 0) return "";
        return responseBody.substring(pos, end);
    };
    String bridgeUrl  = extractStr("bridgeUrl");
    String bridgeName = extractStr("bridgeName");
    String bridgeDesc = extractStr("bridgeDesc");
    if (bridgeUrl.length() == 0) bridgeUrl = "wifi"; // Fallback
    if (bridgeName.length() == 0) bridgeName = "Bridge";

    String out = String("{") + "\"wifiOnline\":true,";
    out += "\"bridgeUrl\":\"" + bridgeUrl + "\",";
    out += "\"bridgeName\":\"" + bridgeName + "\",";
    out += "\"bridgeDesc\":\"" + bridgeDesc + "\",";
    out += "\"modules\":";
    int modStart = responseBody.indexOf("\"modules\":");
    if (modStart >= 0) {
        int arrStart = responseBody.indexOf('[', modStart);
        if (arrStart >= 0) {
            int depth = 0, arrEnd = arrStart;
            for (int i = arrStart; i < (int)responseBody.length(); i++) {
                if (responseBody[i] == '[') depth++;
                else if (responseBody[i] == ']') { depth--; if (depth == 0) { arrEnd = i; break; } }
            }
            out += responseBody.substring(arrStart, arrEnd + 1);
        } else { out += "[]"; }
    } else { out += "[]"; }
    out += "}";

    SimpleHTTPServer::sendJSON(client, out);
}

void FLASHMEM SimpleWebManager::handleEventLoggerConfig(EthernetClient& client, const String& method) {
    EventLogger* logger = EventLogger::getInstance();
    
    if (method == "GET") {
        // Return current configuration
        EventConfig& config = logger->getConfig();
        StaticJsonDocument<256> doc;
        doc["serialEnabled"] = config.enableSerial;
        doc["serialLevel"] = config.serialLevel;
        doc["udpEnabled"] = config.enableUDP;
        doc["udpLevel"] = config.udpLevel;
        doc["rateLimitDisabled"] = config.disableRateLimit;
        
        String json;
        serializeJson(doc, json);
        SimpleHTTPServer::sendJSON(client, json);
        
    } else if (method == "POST") {
        // Read POST body
        String body = readPostBody(client);
        
        LOG_INFO(EventSource::NETWORK, "EventLogger POST body: %s", body.c_str());
        
        // Parse JSON
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            LOG_ERROR(EventSource::NETWORK, "EventLogger JSON parse error: %s", error.c_str());
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }
        
        // Apply EventLogger settings
        if (!doc["serialEnabled"].isNull()) {
            bool enabled = doc["serialEnabled"];
            logger->enableSerial(enabled);
            LOG_INFO(EventSource::NETWORK, "Set serial enabled: %d", enabled);
        }
        if (!doc["udpEnabled"].isNull()) {
            bool enabled = doc["udpEnabled"];
            logger->enableUDP(enabled);
            LOG_INFO(EventSource::NETWORK, "Set UDP enabled: %d", enabled);
        }
        if (!doc["serialLevel"].isNull()) {
            int level = doc["serialLevel"];
            logger->setSerialLevel(static_cast<EventSeverity>(level));
            LOG_INFO(EventSource::NETWORK, "Set serial level: %d", level);
        }
        if (!doc["udpLevel"].isNull()) {
            int level = doc["udpLevel"];
            logger->setUDPLevel(static_cast<EventSeverity>(level));
            LOG_INFO(EventSource::NETWORK, "Set UDP level: %d", level);
        }
        if (!doc["rateLimitDisabled"].isNull()) {
            bool disabled = doc["rateLimitDisabled"];
            logger->setRateLimitEnabled(!disabled);
            LOG_INFO(EventSource::NETWORK, "Set rate limit disabled: %d", disabled);
        }
        
        // Force save after all changes
        logger->saveConfig();
        
        EventConfig& config = logger->getConfig();
        LOG_INFO(EventSource::NETWORK, "EventLogger config after update: Serial=%d/%d, UDP=%d/%d, RateLimit=%d", 
                 config.enableSerial, config.serialLevel, 
                 config.enableUDP, config.udpLevel, 
                 config.disableRateLimit);
        
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"saved\"}");
        
    } else {
        SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
    }
}

void FLASHMEM SimpleWebManager::handleLogViewerData(EthernetClient& client) {
    EventLogger* logger = EventLogger::getInstance();

    // Get log buffer info - capture snapshot to avoid race conditions
    __disable_irq();  // Disable interrupts briefly
    size_t count = logger->getLogBufferCount();
    size_t head = logger->getLogBufferHead();
    size_t bufferSize = logger->getLogBufferSize();

    // Copy buffer entries to avoid corruption during output
    LogEntry snapshot[100];
    const LogEntry* buffer = logger->getLogBuffer();
    memcpy(snapshot, buffer, sizeof(snapshot));
    __enable_irq();  // Re-enable interrupts

    // Send response headers manually for chunked streaming
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();

    // Stream JSON directly to client
    client.print("{\"logs\":[");

    // Read from oldest to newest (circular buffer)
    size_t start = (count < bufferSize) ? 0 : head;

    for (size_t i = 0; i < count; i++) {
        size_t index = (start + i) % bufferSize;
        const LogEntry& entry = snapshot[index];

        if (i > 0) client.print(",");

        client.print("{\"timestamp\":");
        client.print(entry.timestamp);
        client.print(",\"severity\":");
        client.print(static_cast<uint8_t>(entry.severity));
        client.print(",\"source\":");
        client.print(static_cast<uint8_t>(entry.source));
        client.print(",\"message\":\"");

        // Properly escape JSON special characters
        const char* msg = entry.message;
        for (size_t j = 0; msg[j] != '\0'; j++) {
            char c = msg[j];
            switch (c) {
                case '"':  client.print("\\\""); break;
                case '\\': client.print("\\\\"); break;
                case '\n': client.print("\\n"); break;
                case '\r': client.print("\\r"); break;
                case '\t': client.print("\\t"); break;
                case '\b': client.print("\\b"); break;
                case '\f': client.print("\\f"); break;
                default:
                    if (c >= 32 && c < 127) {
                        client.print(c);
                    }
                    // Skip control chars and non-ASCII
                    break;
            }
        }

        client.print("\",\"severityName\":\"");
        client.print(logger->severityToString(entry.severity));
        client.print("\",\"sourceName\":\"");
        client.print(logger->sourceToString(entry.source));
        client.print("\"}");
    }

    client.print("]}");
}

void FLASHMEM SimpleWebManager::handleNetworkConfig(EthernetClient& client, const String& method) {
    if (method == "GET") {
        // Return current IP as array
        IPAddress ip = Ethernet.localIP();
        
        StaticJsonDocument<128> doc;
        JsonArray ipArray = doc.createNestedArray("ip");
        ipArray.add(ip[0]);
        ipArray.add(ip[1]);
        ipArray.add(ip[2]);
        ipArray.add(ip[3]);
        
        String json;
        serializeJson(doc, json);
        SimpleHTTPServer::sendJSON(client, json);
        
    } else if (method == "POST") {
        // Read POST body
        String body = readPostBody(client);
        
        // Parse JSON
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }
        
        // Get IP array
        JsonArray ipArray = doc["ip"];
        if (ipArray.size() >= 3) {
            uint8_t octet1 = ipArray[0];
            uint8_t octet2 = ipArray[1];
            uint8_t octet3 = ipArray[2];
            
            // Update network configuration using ConfigManager
            ConfigManager* config = ConfigManager::getInstance();
            
            // Update IP addresses (keeping 4th octet as 126)
            uint8_t newIP[4] = {octet1, octet2, octet3, 126};
            config->setIPAddress(newIP);
            
            // Update destination IP to broadcast on new subnet
            uint8_t newDest[4] = {octet1, octet2, octet3, 255};
            config->setDestIP(newDest);
            
            // Update gateway to .1 on new subnet
            uint8_t newGateway[4] = {octet1, octet2, octet3, 1};
            config->setGateway(newGateway);
            
            // Save to EEPROM
            config->saveNetworkConfig();
            
            LOG_INFO(EventSource::NETWORK, "Network IP saved: %d.%d.%d.126 (reboot required)", 
                     octet1, octet2, octet3);
            
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"ok\"}");
        } else {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"error\":\"Invalid IP format\"}");
        }
        
    } else {
        SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
    }
}

void FLASHMEM SimpleWebManager::handleDeviceSettings(EthernetClient& client, const String& method) {
    if (method == "GET") {
        // Return current settings from ConfigManager
        ConfigManager* config = ConfigManager::getInstance();
        
        StaticJsonDocument<640> doc;
        doc["deviceType"] = "Steer";  // Fixed for steer module
        doc["moduleId"] = 126;  // Steer module ID
        doc["udpPassthrough"] = config->getGPSPassThrough();
        doc["sensorFusion"] = config->getINSUseFusion();
        doc["pwmBrakeMode"] = config->getPWMBrakeMode();
        doc["pwmFilterAlpha"] = config->getPwmFilterAlpha();
        doc["pwmMinThresholdPct"] = config->getPwmMinThresholdPct();
        doc["softStartDuration"] = config->getSoftStartDurationMs();
        doc["encoderType"] = config->getEncoderType();
        doc["serialRadioBaud"] = config->getSerialRadioBaudRate();
        doc["jdPWMEnabled"] = config->getJDPWMEnabled();
        doc["jdPWMSensitivity"] = config->getJDPWMSensitivity();
        doc["buzzerVolume"] = config->getBuzzerVolume();
        doc["sectionControlActive"] = !config->getSectionControlSleepMode();
        doc["moduleShortname"]   = config->getModuleShortname();
        doc["moduleLongname"]    = config->getModuleLongname();
        doc["moduleDescription"] = config->getModuleDescription();

        String json;
        serializeJson(doc, json);
        SimpleHTTPServer::sendJSON(client, json);
        
    } else if (method == "POST") {
        // Read POST body
        String body = readPostBody(client);
        
        // Parse JSON
        StaticJsonDocument<640> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }
        
        // Extract settings from JSON
        bool udpPassthrough = doc["udpPassthrough"] | false;
        bool sensorFusion = doc["sensorFusion"] | false;
        bool pwmBrakeMode = doc["pwmBrakeMode"] | false;
        uint8_t pwmFilterAlpha = (uint8_t)(doc["pwmFilterAlpha"] | 90);
        uint8_t pwmMinThresholdPct = (uint8_t)(doc["pwmMinThresholdPct"] | 25);
        uint16_t softStartDuration = doc["softStartDuration"] | 500;
        int encoderType = doc["encoderType"] | 1;
        uint32_t serialRadioBaud = doc["serialRadioBaud"] | 115200;
        bool jdPWMEnabled = doc["jdPWMEnabled"] | false;
        int jdPWMSensitivity = doc["jdPWMSensitivity"] | 5;
        int buzzerVolume = doc["buzzerVolume"] | 1;
        bool sectionControlActive;
        if (doc.containsKey("sectionControlActive")) {
            sectionControlActive = doc["sectionControlActive"] | true;
        } else {
            // Backward compatibility for older UI payloads.
            bool sectionControlSleepMode = doc["sectionControlSleepMode"] | false;
            sectionControlActive = !sectionControlSleepMode;
        }

        // Save to ConfigManager
        ConfigManager* config = ConfigManager::getInstance();
        config->setGPSPassThrough(udpPassthrough);
        config->setPWMBrakeMode(pwmBrakeMode);
        config->setPwmFilterAlpha(constrain(pwmFilterAlpha, 0, 97));
        config->setPwmMinThresholdPct(constrain(pwmMinThresholdPct, 0, 100));
        config->setSoftStartDurationMs(softStartDuration);
        config->setEncoderType(encoderType);
        config->setSerialRadioBaudRate(serialRadioBaud);
        config->setJDPWMEnabled(jdPWMEnabled);
        config->setJDPWMSensitivity(jdPWMSensitivity);
        config->setBuzzerVolume(buzzerVolume);
        config->setSectionControlSleepMode(!sectionControlActive);

        // Module identification (optional in payload; only update if provided)
        if (doc.containsKey("moduleShortname"))   config->setModuleShortname(doc["moduleShortname"] | "");
        if (doc.containsKey("moduleLongname"))     config->setModuleLongname(doc["moduleLongname"] | "");
        if (doc.containsKey("moduleDescription")) config->setModuleDescription(doc["moduleDescription"] | "");

        // Save to EEPROM
        config->setINSUseFusion(sensorFusion);
        config->saveINSConfig();         // This saves VWAS / sensor fusion setting
        config->saveTurnSensorConfig();  // This saves encoder type and JD PWM settings
        config->saveSteerConfig();       // This saves PWM brake mode
        config->saveGPSConfig();         // This saves GPS passthrough
        config->saveMiscConfig();        // This saves buzzer volume, LED, PWM filter settings
        config->saveMachineConfig();     // This saves section control sleep mode
        config->saveModuleIdentConfig(); // This saves shortname / longname / description
        
        // Apply JD PWM mode change to ADProcessor
        extern ADProcessor adProcessor;
        adProcessor.setJDPWMMode(jdPWMEnabled);

        // Update GNSSProcessor with new passthrough setting
        gnssProcessor.setUDPPassthrough(udpPassthrough);

        // Apply new radio baud rate immediately
        extern SerialManager serialManager;
        serialManager.updateRadioBaudRate(serialRadioBaud);

        // Apply soft start duration to AutosteerProcessor immediately
        // Also re-initialize VWAS if the setting changed
        AutosteerProcessor* autosteerProc = AutosteerProcessor::getInstance();
        if (autosteerProc) {
            autosteerProc->setSoftStartDuration(softStartDuration);
            autosteerProc->setSoftAccelDuration(softStartDuration / 2);
            LOG_INFO(EventSource::NETWORK, "Soft start APPLIED: %dms (accel=%dms) - verifying: getSoftStartDuration()=%d",
                     softStartDuration, softStartDuration / 2, autosteerProc->getSoftStartDuration());
            // Re-init to pick up VWAS enable/disable change
            autosteerProc->init();
        } else {
            LOG_ERROR(EventSource::NETWORK, "Failed to apply soft start - AutosteerProcessor not available!");
        }

        LOG_DEBUG(EventSource::NETWORK, "Device settings saved: UDP=%d, Brake=%d, SoftStart=%d, Encoder=%d, RadioBaud=%lu",
                  udpPassthrough, pwmBrakeMode, softStartDuration, encoderType, serialRadioBaud);

        SimpleHTTPServer::sendJSON(client, "{\"status\":\"saved\"}");
        
    } else {
        SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
    }
}

void FLASHMEM SimpleWebManager::handleAnalogWorkSwitchStatus(EthernetClient& client) {
    LOG_DEBUG(EventSource::NETWORK, "Analog work switch status requested");
    ADProcessor* adProc = ADProcessor::getInstance();
    if (!adProc) {
        LOG_ERROR(EventSource::NETWORK, "ADProcessor not available");
        SimpleHTTPServer::send(client, 503, "application/json", "{\"error\":\"ADProcessor not available\"}");
        return;
    }
    
    StaticJsonDocument<256> doc;
    doc["enabled"] = adProc->isAnalogWorkSwitchEnabled();
    doc["setpoint"] = (int)round(adProc->getWorkSwitchSetpoint());
    doc["hysteresis"] = (int)round(adProc->getWorkSwitchHysteresis());
    doc["invert"] = adProc->getInvertWorkSwitch();
    doc["percent"] = adProc->getWorkSwitchAnalogPercent();
    doc["state"] = adProc->isWorkSwitchOn();
    doc["raw"] = adProc->getWorkSwitchAnalogRaw();
    
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

void FLASHMEM SimpleWebManager::handleAnalogWorkSwitchConfig(EthernetClient& client) {
    // Read POST body
    String body = readPostBody(client);
    
    // Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
    }
    
    ADProcessor* adProc = ADProcessor::getInstance();
    if (!adProc) {
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"ADProcessor not available\"}");
        return;
    }
    
    // Update settings if provided
    if (!doc["enabled"].isNull()) {
        adProc->setAnalogWorkSwitchEnabled(doc["enabled"]);
    }
    if (!doc["hysteresis"].isNull()) {
        adProc->setWorkSwitchHysteresis(doc["hysteresis"]);
    }
    if (!doc["invert"].isNull()) {
        adProc->setInvertWorkSwitch(doc["invert"]);
    }
    
    LOG_INFO(EventSource::NETWORK, "Analog work switch config updated");
    
    SimpleHTTPServer::sendJSON(client, "{\"status\":\"saved\"}");
}

void FLASHMEM SimpleWebManager::handleAnalogWorkSwitchSetpoint(EthernetClient& client) {
    ADProcessor* adProc = ADProcessor::getInstance();
    if (!adProc) {
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"ADProcessor not available\"}");
        return;
    }
    
    // Set current reading as new setpoint
    float currentPercent = adProc->getWorkSwitchAnalogPercent();
    adProc->setWorkSwitchSetpoint(currentPercent);
    
    StaticJsonDocument<128> doc;
    doc["status"] = "saved";
    doc["newSetpoint"] = (int)round(currentPercent);
    
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
    
    LOG_INFO(EventSource::NETWORK, "Analog work switch setpoint set to %.1f%%", currentPercent);
}

void FLASHMEM SimpleWebManager::handleOTAUpload(EthernetClient& client) {
    // OTA upload request received
    
    // Initialize OTA handler if needed
    static bool otaInitialized = false;
    if (!otaInitialized) {
        // Initialize OTA handler
        if (!SimpleOTAHandler::init()) {
            LOG_ERROR(EventSource::NETWORK, "OTA init failed");
            SimpleHTTPServer::send(client, 500, "text/plain", "OTA init failed");
            return;
        }
        otaInitialized = true;
    }
    
    // Reset OTA handler for new upload
    SimpleOTAHandler::reset();
    
    // Read hex data from client
    
    // Read data in chunks directly from client
    uint8_t buffer[1024];
    size_t totalBytes = 0;
    bool foundStart = false;
    
    // Set a longer timeout for large files
    unsigned long timeout = 30000; // 30 seconds
    unsigned long start = millis();
    unsigned long lastDataTime = millis();
    
    while (client.connected() && (millis() - start < timeout) && !SimpleOTAHandler::isComplete()) {
        if (client.available()) {
            size_t bytesRead = client.readBytes(buffer, sizeof(buffer));
            
            if (bytesRead > 0) {
                lastDataTime = millis(); // Track when we last received data
                
                // Check for hex file start on first chunk
                if (!foundStart) {
                    // Look for ':' which starts a hex file
                    for (size_t i = 0; i < bytesRead; i++) {
                        if (buffer[i] == ':') {
                            foundStart = true;
                            // Process from this point
                            if (!SimpleOTAHandler::processChunk(&buffer[i], bytesRead - i)) {
                                const char* error = SimpleOTAHandler::getError();
                                LOG_ERROR(EventSource::NETWORK, "OTA processing failed: %s", error ? error : "Unknown error");
                                SimpleHTTPServer::send(client, 400, "text/plain", error ? error : "Processing failed");
                                return;
                            }
                            totalBytes += (bytesRead - i);
                            break;
                        }
                    }
                    if (!foundStart) {
                        LOG_ERROR(EventSource::NETWORK, "No hex data found in first chunk");
                        SimpleHTTPServer::send(client, 400, "text/plain", "Invalid hex file format");
                        return;
                    }
                } else {
                    // Process subsequent chunks
                    if (!SimpleOTAHandler::processChunk(buffer, bytesRead)) {
                        const char* error = SimpleOTAHandler::getError();
                        LOG_ERROR(EventSource::NETWORK, "OTA processing failed: %s", error ? error : "Unknown error");
                        SimpleHTTPServer::send(client, 400, "text/plain", error ? error : "Processing failed");
                        return;
                    }
                    totalBytes += bytesRead;
                }
                
                // Reset timeout on data received
                start = millis();
                
                // Log progress every 10KB
                if (totalBytes % 10240 < 1024) {
                    // Progress: totalBytes
                }
                
                // Check if OTA is complete
                if (SimpleOTAHandler::isComplete()) {
                    // Upload complete
                    break;
                }
            }
        } else {
            // No data available
            if (foundStart && millis() - lastDataTime > 1000) {
                // If we've started receiving data but haven't gotten any new data for 1 second,
                // assume the upload is complete
                // No data for 1 second, assuming complete
                break;
            }
            
            delay(1);
            
            // Also check for completion during idle time
            if (SimpleOTAHandler::isComplete()) {
                // Upload complete during idle
                break;
            }
        }
    }
    
    LOG_INFO(EventSource::NETWORK, "Received %lu total bytes", totalBytes);
    
    if (totalBytes == 0) {
        LOG_ERROR(EventSource::NETWORK, "No data received");
        SimpleHTTPServer::send(client, 400, "text/plain", "No data received");
        return;
    }
    
    // Check if we need to finalize even without explicit EOF
    if (!SimpleOTAHandler::isComplete() && foundStart) {
        // No EOF record found, check if valid
        // Process any remaining data in buffer
        if (SimpleOTAHandler::processChunk((const uint8_t*)"\n", 1)) {
            // Force a newline to process any pending hex line
        }
    }
    
    // Finalize OTA upload
    
    // Finalize the upload
    if (SimpleOTAHandler::finalize()) {
        LOG_INFO(EventSource::NETWORK, "OTA upload successful, sending response");
        
        // Send a quick success response
        SimpleHTTPServer::send(client, 200, "text/plain", "OK");
        
        // Make sure the response is fully sent
        client.flush();
        client.stop();  // Close the connection cleanly
        
        // Small delay to ensure TCP FIN is sent
        delay(100);
        
        // Apply the update
        LOG_INFO(EventSource::NETWORK, "Applying firmware update now");
        SimpleOTAHandler::applyUpdate();
    } else {
        const char* error = SimpleOTAHandler::getError();
        LOG_ERROR(EventSource::NETWORK, "OTA finalization failed: %s", error ? error : "Unknown error");
        SimpleHTTPServer::send(client, 400, "text/plain", error ? error : "Finalization failed");
    }
}

// Helper methods

// -----------------------------------------------------------------------
// DNS Alias page and API
// -----------------------------------------------------------------------

void SimpleWebManager::sendDNSAliasPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_DNS_ALIAS_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_DNS_ALIAS_PAGE);
}

void FLASHMEM SimpleWebManager::handleDNSAliasConfig(EthernetClient& client, const String& method) {
    ConfigManager* config = ConfigManager::getInstance();
    if (!config) {
        SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Config unavailable\"}");
        return;
    }

    if (method == "GET") {
        // Return current aliases as JSON array
        String json = "{\"aliases\":[";
        for (uint8_t i = 0; i < 4; i++) {
            if (i > 0) json += ",";
            json += "\"";
            json += config->getDNSAlias(i);
            json += "\"";
        }
        json += "]}";
        SimpleHTTPServer::sendJSON(client, json);

    } else if (method == "POST") {
        String body = readPostBody(client);

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);
        if (error) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        JsonArray arr = doc["aliases"];
        if (!arr) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Missing aliases array\"}");
            return;
        }

        // Save aliases; clear owned-hostname list and rebuild it
        for (uint8_t i = 0; i < 4; i++) {
            const char* val = (i < arr.size()) ? (const char*)arr[i] : "";
            // Sanitize: only lowercase alphanumeric and hyphen, max 11 chars
            char sanitized[12] = {};
            int slen = 0;
            for (int k = 0; val[k] && slen < 11; k++) {
                char c = (char)tolower((unsigned char)val[k]);
                if (isalnum((unsigned char)c) || c == '-') sanitized[slen++] = c;
            }
            sanitized[slen] = '\0';
            config->setDNSAlias(i, sanitized);
        }
        config->saveDNSAliasConfig();

        // Rebuild ownedHostnames in HTTP server from the newly saved aliases
        for (uint8_t i = 0; i < 4; i++) {
            const char* alias = config->getDNSAlias(i);
            if (alias && alias[0] != '\0') {
                httpServer.addOwnedHostname(String(alias));
            }
        }

        LOG_INFO(EventSource::NETWORK, "DNS aliases updated: '%s','%s','%s','%s'",
                 config->getDNSAlias(0), config->getDNSAlias(1),
                 config->getDNSAlias(2), config->getDNSAlias(3));

        SimpleHTTPServer::sendJSON(client, "{\"status\":\"ok\"}");

    } else {
        SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
    }
}

String FLASHMEM SimpleWebManager::readPostBody(EthernetClient& client) {
    String body;
    body.reserve(20480); // Pre-allocate for 20KB (handles JSON config files)

    // Note: SimpleHTTPServer should have already consumed headers
    // Just read any remaining data
    int timeout = 2000; // 2 second timeout for large files
    unsigned long start = millis();
    unsigned long lastDataTime = millis();

    while (millis() - start < timeout) {
        while (client.available()) {
            // Read in chunks for better performance
            char buffer[512];
            size_t bytesRead = client.readBytes(buffer, sizeof(buffer));
            if (bytesRead > 0) {
                // Temporarily null-terminate and append
                char tempChar = buffer[bytesRead];
                buffer[bytesRead] = '\0';
                body += buffer;
                buffer[bytesRead] = tempChar;
                lastDataTime = millis(); // Reset timeout on data
            }
        }

        // If we have data and haven't received anything for 50ms, we're done
        if (body.length() > 0 && (millis() - lastDataTime > 50)) {
            break;
        }

        delay(1); // Small delay to yield
    }

    return body;
}

String FLASHMEM SimpleWebManager::buildLevelOptions(uint8_t selectedLevel) {
    String options;
    // Use proper syslog severity levels
    const char* levels[] = {"EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"};
    const uint8_t values[] = {0, 1, 2, 3, 4, 5, 6, 7};
    
    for (uint8_t i = 0; i < 8; i++) {
        options += "<option value='" + String(values[i]) + "'";
        if (values[i] == selectedLevel) {
            options += " selected";
        }
        options += ">" + String(levels[i]) + "</option>";
    }
    
    return options;
}

// Telemetry broadcast

void FLASHMEM SimpleWebManager::broadcastTelemetry() {
    // Track connection state
    static uint32_t connectionStart = 0;
    static bool connectionPrimed = false;
    static size_t lastClientCount = 0;

    size_t currentClientCount = telemetryWS.getClientCount();

    // Early exit if no clients connected
    if (currentClientCount == 0) {
        connectionStart = 0;
        connectionPrimed = false;
        lastClientCount = 0;
        return;
    }

    // Now called by SimpleScheduler at 100Hz
    uint32_t now = millis();

    // Check for new connection
    if (currentClientCount > 0 && lastClientCount == 0) {
        connectionStart = now;
        connectionPrimed = false;
        LOG_DEBUG(EventSource::NETWORK, "WebSocket client connected, priming connection");
    }
    lastClientCount = currentClientCount;

    // During connection priming (first 5 seconds), send extra updates
    // SimpleScheduler calls us at 100Hz, but we can send more frequently during priming
    if (!connectionPrimed && now - connectionStart < 5000) {
        // Send immediately during priming period (up to 100Hz from scheduler)
        connectionPrimed = (now - connectionStart >= 5000);
    } else if (!connectionPrimed) {
        connectionPrimed = true;
    }
    
    // Build telemetry packet
    TelemetryPacket packet;
    packet.timestamp = now;
    
    // Get WAS data
    ADProcessor* adProc = ADProcessor::getInstance();
    if (adProc) {
        packet.was_angle = adProc->getWASAngle();
        packet.was_angle_target = AutosteerProcessor::getInstance()->getTargetAngle();
        packet.current_draw = adProc->getMotorCurrent() / 1000.0f;  // Convert mA to A
    } else {
        packet.was_angle = 0;
        packet.was_angle_target = 0;
        packet.current_draw = 0;
    }
    
    // Get encoder data
    if (encoderProcessor) {
        packet.encoder_count = encoderProcessor->getPulseCount();
    } else {
        packet.encoder_count = 0;
    }
    
    // Get switch states from ADProcessor
    if (adProc) {
        packet.steer_switch = adProc->isSteerSwitchOn() ? 1 : 0;
        packet.work_switch = adProc->isWorkSwitchOn() ? 1 : 0;
        packet.work_analog_percent = (uint8_t)round(adProc->getWorkSwitchAnalogPercent());
    } else {
        packet.steer_switch = 0;
        packet.work_switch = 0;
        packet.work_analog_percent = 0;
    }
    
    // Get speed and heading from GNSSProcessor
    const auto& gpsData = gnssProcessor.getData();
    if (gpsData.isValid) {
        packet.speed_kph = gpsData.speedKnots * 1.852f;  // Convert knots to km/h
        packet.heading = gpsData.headingTrue;
    } else {
        packet.speed_kph = 0;
        packet.heading = 0;
    }
    
    // Status flags
    packet.status_flags = 0;
    if (AutosteerProcessor::getInstance()->isEnabled()) {
        packet.status_flags |= 0x01;  // Bit 0: Autosteer enabled
    }
    
    packet.reserved[0] = 0;
    
    // Broadcast to all connected clients
    telemetryWS.broadcastBinary((const uint8_t*)&packet, sizeof(packet));
}

// UM98x GPS Configuration handlers

void SimpleWebManager::sendUM98xConfigPage(EthernetClient& client) {
    extern const char TOUCH_FRIENDLY_GPS_CONFIG_PAGE[];
    SimpleHTTPServer::sendP(client, 200, "text/html; charset=utf-8", TOUCH_FRIENDLY_GPS_CONFIG_PAGE);
}

void FLASHMEM SimpleWebManager::handleUM98xRead(EthernetClient& client) {
    LOG_INFO(EventSource::NETWORK, "handleUM98xRead() called");
    
    // Create UM98xManager instance
    static UM98xManager um98xManager;
    static bool managerInitialized = false;
    
    if (!managerInitialized) {
        // Initialize with GPS1 serial port (SerialGPS1)
        if (!um98xManager.init(&SerialGPS1)) {
            StaticJsonDocument<128> doc;
            doc["success"] = false;
            doc["error"] = "Failed to initialize UM98x manager";
            
            String response;
            serializeJson(doc, response);
            SimpleHTTPServer::send(client, 500, "application/json", response);
            return;
        }
        managerInitialized = true;
    }
    
    // Read configuration
    UM98xManager::UM98xConfig config;
    bool success = um98xManager.readConfiguration(config);
    
    // Build JSON response
    StaticJsonDocument<2048> doc;
    doc["success"] = success;
    
    if (success) {
        doc["config"] = config.configCommands;
        doc["mode"] = config.modeSettings;
        doc["messages"] = config.messageSettings;
    } else {
        doc["error"] = "Failed to read GPS configuration";
    }
    
    String response;
    serializeJson(doc, response);
    SimpleHTTPServer::send(client, success ? 200 : 500, "application/json", response);
}

void FLASHMEM SimpleWebManager::handleUM98xWrite(EthernetClient& client) {
    LOG_INFO(EventSource::NETWORK, "handleUM98xWrite() called");
    
    // Parse POST body
    String body = readPostBody(client);
    
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
        StaticJsonDocument<128> responseDoc;
        responseDoc["success"] = false;
        responseDoc["error"] = "Invalid JSON";
        
        String response;
        serializeJson(responseDoc, response);
        SimpleHTTPServer::send(client, 400, "application/json", response);
        return;
    }
    
    // Create UM98xManager instance
    static UM98xManager um98xManager;
    static bool managerInitialized = false;
    
    if (!managerInitialized) {
        // Initialize with GPS1 serial port (SerialGPS1)
        if (!um98xManager.init(&SerialGPS1)) {
            StaticJsonDocument<128> responseDoc;
            responseDoc["success"] = false;
            responseDoc["error"] = "Failed to initialize UM98x manager";
            
            String response;
            serializeJson(responseDoc, response);
            SimpleHTTPServer::send(client, 500, "application/json", response);
            return;
        }
        managerInitialized = true;
    }
    
    // Extract configuration from JSON
    UM98xManager::UM98xConfig config;
    config.configCommands = doc["config"] | "";
    config.modeSettings = doc["mode"] | "";
    config.messageSettings = doc["messages"] | "";
    
    // Write configuration
    bool success = um98xManager.writeConfiguration(config);
    
    // Build response
    StaticJsonDocument<128> responseDoc;
    responseDoc["success"] = success;
    if (!success) {
        responseDoc["error"] = "Failed to write GPS configuration";
    }
    
    String response;
    serializeJson(responseDoc, response);
    SimpleHTTPServer::send(client, success ? 200 : 500, "application/json", response);
}

void FLASHMEM SimpleWebManager::handleCANInfo(EthernetClient& client) {
    // Check if custom configuration exists in LittleFS
    if (CANConfigStorage::hasCustomConfig()) {
        // Stream custom configuration directly from flash to save RAM
        LittleFS_Program& fs = CANConfigStorage::getFS();
        File file = fs.open("/can_config.json", FILE_READ);

        if (file && file.size() > 0) {
            // Send HTTP headers
            client.print("HTTP/1.1 200 OK\r\n");
            client.print("Content-Type: application/json\r\n");
            client.print("Connection: close\r\n");
            client.print("\r\n");

            // Stream file in chunks to avoid loading entire file into RAM
            uint8_t buffer[512];
            while (file.available()) {
                size_t bytesRead = file.read(buffer, sizeof(buffer));
                if (bytesRead > 0) {
                    size_t written = client.write(buffer, bytesRead);

                    // If write returned 0, buffer is full - wait for it to drain
                    if (written == 0) {
                        client.flush();
                        delay(10);
                        written = client.write(buffer, bytesRead);
                    }

                    // Pace writes to prevent buffer overflow
                    if (file.position() % 2048 == 0) {
                        client.flush();
                        delay(1);
                    }
                }
            }
            file.close();
            client.flush();
            return;
        }

        if (file) file.close();
    }

    // Fallback to default configuration from PROGMEM
    SimpleHTTPServer::sendP(client, 200, "application/json", CAN_INFO_JSON);
}

void FLASHMEM SimpleWebManager::handleCANConfig(EthernetClient& client, const String& method) {
    extern ConfigManager configManager;

    if (method == "GET") {
        // Return current CAN configuration
        CANSteerConfig config = configManager.getCANSteerConfig();

        StaticJsonDocument<512> doc;
        doc["brand"] = config.brand;
        doc["can1Speed"] = config.can1Speed;
        doc["can1Function"] = config.can1Function;
        doc["can1Name"] = config.can1Name;
        doc["can2Speed"] = config.can2Speed;
        doc["can2Function"] = config.can2Function;
        doc["can2Name"] = config.can2Name;
        doc["can3Speed"] = config.can3Speed;
        doc["can3Function"] = config.can3Function;
        doc["can3Name"] = config.can3Name;
        doc["moduleID"] = config.moduleID;
        doc["keyaWasSource"] = config.reserved[0];  // 0=Analog, 1=CAN curve, 2=Virtual (fusion)

        String json;
        serializeJson(doc, json);
        SimpleHTTPServer::sendJSON(client, json);

    } else if (method == "POST") {
        // Read POST body
        String body = readPostBody(client);

        LOG_INFO(EventSource::NETWORK, "CAN config POST body: %s", body.c_str());

        // Parse JSON
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            LOG_ERROR(EventSource::NETWORK, "CAN config JSON parse error: %s", error.c_str());
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        // Get current config
        CANSteerConfig config = configManager.getCANSteerConfig();

        // Update config from JSON
        if (!doc["brand"].isNull()) {
            config.brand = doc["brand"];
        }
        if (!doc["can1Speed"].isNull()) {
            config.can1Speed = doc["can1Speed"];
        }
        if (!doc["can1Function"].isNull()) {
            config.can1Function = doc["can1Function"];
        }
        if (!doc["can1Name"].isNull()) {
            config.can1Name = doc["can1Name"];
        }
        if (!doc["can2Speed"].isNull()) {
            config.can2Speed = doc["can2Speed"];
        }
        if (!doc["can2Function"].isNull()) {
            config.can2Function = doc["can2Function"];
        }
        if (!doc["can2Name"].isNull()) {
            config.can2Name = doc["can2Name"];
        }
        if (!doc["can3Speed"].isNull()) {
            config.can3Speed = doc["can3Speed"];
        }
        if (!doc["can3Function"].isNull()) {
            config.can3Function = doc["can3Function"];
        }
        if (!doc["can3Name"].isNull()) {
            config.can3Name = doc["can3Name"];
        }
        if (!doc["moduleID"].isNull()) {
            config.moduleID = doc["moduleID"];
        }
        if (!doc["keyaWasSource"].isNull()) {
            int src = doc["keyaWasSource"];
            config.reserved[0] = (uint8_t)constrain(src, 0, 2);
        }

        // Validate: ensure no duplicate bus names (except None/0)
        uint8_t busNames[3] = {config.can1Name, config.can2Name, config.can3Name};
        for (int i = 0; i < 3; i++) {
            if (busNames[i] != 0) {  // Skip "None"
                for (int j = i + 1; j < 3; j++) {
                    if (busNames[i] == busNames[j]) {
                        LOG_ERROR(EventSource::NETWORK, "Duplicate bus name detected: CAN%d and CAN%d both set to %d",
                                  i+1, j+1, busNames[i]);
                        SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Each bus name can only be used once\"}");
                        return;
                    }
                }
            }
        }

        // Save to EEPROM
        configManager.setCANSteerConfig(config);
        configManager.saveCANSteerConfig();

        // "Virtuell" only takes effect if VWAS itself is enabled - auto-enable it so the CAN
        // WAS-source selector and the separate device-settings VWAS checkbox don't have to be
        // kept in sync manually.
        if (config.reserved[0] == 2 && !configManager.getINSUseFusion()) {
            configManager.setINSUseFusion(true);
            configManager.saveINSConfig();
            AutosteerProcessor* autosteerProc = AutosteerProcessor::getInstance();
            if (autosteerProc) {
                autosteerProc->init();
            }
            LOG_INFO(EventSource::NETWORK, "Virtual WAS source selected - auto-enabled VWAS");
        }

        LOG_INFO(EventSource::NETWORK, "CAN config saved - Brand: %d, CAN1: %d/%d, CAN2: %d/%d, CAN3: %d/%d",
                 config.brand, config.can1Speed, config.can1Function,
                 config.can2Speed, config.can2Function,
                 config.can3Speed, config.can3Function);

        SimpleHTTPServer::sendJSON(client, "{\"status\":\"ok\",\"message\":\"Configuration saved. Restart required.\"}");
    } else {
        SimpleHTTPServer::send(client, 405, "text/plain", "Method Not Allowed");
    }
}

void FLASHMEM SimpleWebManager::handleCANConfigUpload(EthernetClient& client) {
    // Read JSON content from POST body
    String jsonContent = readPostBody(client);

    LOG_INFO(EventSource::NETWORK, "CAN config upload - received %d bytes", jsonContent.length());

    if (jsonContent.length() == 0) {
        SimpleHTTPServer::send(client, 400, "text/plain", "No content received");
        return;
    }

    // Validate JSON by parsing it - use DynamicJsonDocument for large files
    // Increase nesting depth for complex JSON structures (default is 10)
    DynamicJsonDocument testDoc(24576);  // 24KB buffer for validation
    DeserializationError error = deserializeJson(testDoc, jsonContent, DeserializationOption::NestingLimit(20));

    if (error) {
        String errorMsg = "Invalid JSON: ";
        errorMsg += error.c_str();
        SimpleHTTPServer::send(client, 400, "text/plain", errorMsg);
        LOG_ERROR(EventSource::NETWORK, "CAN config upload - invalid JSON: %s", error.c_str());
        return;
    }

    // Write to LittleFS
    if (CANConfigStorage::writeCustomConfig(jsonContent)) {
        LOG_INFO(EventSource::NETWORK, "Custom CAN config uploaded (%d bytes)", jsonContent.length());
        SimpleHTTPServer::send(client, 200, "text/plain", "Upload successful");
    } else {
        LOG_ERROR(EventSource::NETWORK, "Failed to write custom CAN config to flash");
        SimpleHTTPServer::send(client, 500, "text/plain", "Failed to write to flash");
    }
}

void FLASHMEM SimpleWebManager::handleCANConfigRestore(EthernetClient& client) {
    // Delete custom configuration to restore default
    if (CANConfigStorage::deleteCustomConfig()) {
        LOG_INFO(EventSource::NETWORK, "Custom CAN config deleted, restored to default");
        SimpleHTTPServer::send(client, 200, "text/plain", "Default configuration restored");
    } else {
        LOG_ERROR(EventSource::NETWORK, "Failed to delete custom CAN config");
        SimpleHTTPServer::send(client, 500, "text/plain", "Failed to restore default");
    }
}

void FLASHMEM SimpleWebManager::logPeriodicStatus() {
    telemetryWS.logPeriodicStatus();
}

void FLASHMEM SimpleWebManager::handleCANConfigStatus(EthernetClient& client) {
    StaticJsonDocument<256> doc;

    if (CANConfigStorage::hasCustomConfig()) {
        doc["custom"] = true;

        // Try to get version from custom config
        String customConfig = CANConfigStorage::readCustomConfig();
        if (customConfig.length() > 0) {
            StaticJsonDocument<512> configDoc;
            DeserializationError error = deserializeJson(configDoc, customConfig);
            if (!error && !configDoc["version"].isNull()) {
                doc["version"] = configDoc["version"].as<String>();
            } else {
                doc["version"] = "Unknown";
            }
            doc["size"] = CANConfigStorage::getCustomConfigSize();
        } else {
            doc["version"] = "Error reading";
            doc["size"] = 0;
        }
    } else {
        doc["custom"] = false;
        doc["version"] = "2.0";  // Default version from PROGMEM
    }

    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

// ============================================================================
// Virtual WAS (VWAS) API handlers
// ============================================================================

void FLASHMEM SimpleWebManager::handleVWASStatus(EthernetClient& client) {
    if (!wheelAngleFusionPtr) {
        SimpleHTTPServer::send(client, 503, "application/json",
                                "{\"error\":\"VWAS not active\"}");
        return;
    }
    // Live values + calibration state + roll + the AgOpenGPS scale/offset pair +
    // the zero-point observer status. The duplicated JSON key strings dominate the
    // cost; 1024 was already close before the six observer keys were added, and the
    // P2 gain keys pushed it further, so the capacity is raised rather than left to
    // silently truncate (a truncated document is indistinguishable from broken
    // buttons in the UI).
    // Raised again from 1600 to 2400 for the HeadingBridge keys, and to 2800 with
    // the projection/speed-scale keys - a truncated document would hide exactly the
    // diagnostics this page exists for.
    StaticJsonDocument<3200> doc;
    doc["encoderAngle"] = wheelAngleFusionPtr->getEncoderAngle();
    doc["fusedAngle"] = wheelAngleFusionPtr->getFusedAngle();
    doc["gpsAngle"] = wheelAngleFusionPtr->getGPSAngle();
    doc["gpsAngleValid"] = wheelAngleFusionPtr->hasValidGPSAngle();
    // Raw calibration data (Phase 2): raw counts + straight-ahead reference,
    // so the web UI can show live values while the user turns the wheel.
    // Calibration state: the boot log warns loudly when this is false, so the
    // web UI must show it too - otherwise the warning scrolls past unseen.
    // Roll from the source currently providing the heading. Reported only -
    // AgOpenGPS applies its own sideHillCompFactor to the setpoint.
    doc["roll"] = wheelAngleFusionPtr->getRollDeg();
    doc["rollValid"] = wheelAngleFusionPtr->hasRoll();

    // --- Fahrzeug-Setup-Block: Zustand direkt aus dem EEPROM (2026-11-02) ------
    // Ohne serielle Konsole war bisher nicht feststellbar, ob der gemeinsame
    // Block gueltig ist, verworfen wurde oder nie geschrieben wurde - genau die
    // Frage direkt nach einem Update, wenn das Boot-Log nicht mitgeschnitten
    // wurde.
    //
    // Lesart:
    //   setupMagic = 65535 (0xFFFF) -> EEPROM nie geschrieben (Werkzustand)
    //   setupMagic = 42257 (0xA511) -> unser Block
    //   setupValid = true           -> Kopf, Laenge und CRC passen, die Firmware
    //                                  hat die Werte beim Boot angewendet
    //   setupValid = false          -> beim naechsten Boot werden Defaults
    //                                  geschrieben (vehicle.xml danach importieren)
    {
        AutosteerProcessor* as = AutosteerProcessor::getInstance();
        if (as) {
            const AutosteerProcessor::VehicleSetupState st = as->getVehicleSetupState();
            doc["setupMagic"]     = st.magic;
            doc["setupVer"]       = st.layoutVer;
            doc["setupLen"]       = st.payloadLen;
            doc["setupLenExpect"] = st.expectedLen;
            doc["setupCrcOk"]     = st.crcOk;
            doc["setupValid"]     = st.valid;
        }
    }
    // Ackermann comes from AgOpenGPS (PGN 252), NOT from the VWAS config any
    // more - see decision 1b. Shown so the operator can see which factor is
    // actually being applied to left angles.
    {
        extern ConfigManager configManager;
        // Plan 1: AgOpenGPS owns the VWAS scale and zero offset. The web UI has
        // to show BOTH numbers, otherwise the operator sees a measured cpd in
        // this page and a different one in AOG with no way to tell which is in
        // use. The effective value is what the encoder conversion actually uses.
        // Sign convention (plan 1): the mechanical encoder direction XOR AgOpenGPS's
        // "Invert WAS". Shown because a wrong sign makes the steering run away, and
        // until now the VWAS silently ignored AgOpenGPS's flag.

        // Zero-point observer (Stufe 2). Reported even while it is blocked, so the
        // operator can see what the firmware WOULD correct and how credible the
        // measurement is - the trim must never be the only evidence that it ran.
        const WheelAngleFusion::ZeroObserverStatus zs = wheelAngleFusionPtr->getZeroObserverStatus();
        doc["zeroResidualDeg"] = zs.residualDeg;
        // P2: the gain is configured, not learned. The UI has to show all three
        // numbers - configured magnitude (AgOpenGPS convention), the value actually
        // used including the derived sign, and what the diagnostic estimator reports.
        // A reported value far from the effective one, or of the opposite sign, is
        // the fault that must be visible without a rebuild.
        doc["zeroState"] = zs.state;
        doc["zeroFits"] = zs.fits;
        // The mean roll the gain acted on in the last fit. Without it the log cannot
        // separate a gain error from a real zero movement: errEst must be invariant in
        // the roll, and that is only checkable if the roll is recorded next to it.
        // The rate is the answer to "why is it not converging?" - it is scaled by
        // heading quality and by the roll spread of the last fit, so it can be
        // near-zero without anything being broken.
        doc["zeroRate"] = zs.acquireRateDegPerSec;
    }

    // Heading/roll fall-back (HeadingBridge). Shown so the operator can read the
    // two numbers that belong into other tools:
    //   * mountOffsetMeasuredDeg -> enter -mountOffsetMeasuredDeg as AgOpenGPS's
    //     "dual heading offset" (AOG adds the offset to the received azimuth),
    //   * baselineLengthM / baselineDevPct -> the receiver's own attitude quality
    //     indicator (solved vs mechanically measured antenna distance); the
    //     mechanical value has to be entered so the check can judge it.
    // state/source say whether the published heading is the attitude, a position
    // derived direction or pure dead reckoning.
    {
        const HeadingBridge::Status& bs = headingBridge.getStatus();
        doc["bridgeState"] = HeadingBridge::stateName(bs.state);
        doc["bridgeSource"] = HeadingBridge::sourceName(bs.source);
        doc["bridgeSigmaDeg"] = bs.sigmaDeg;
        doc["bridgeAttAgeMs"] = bs.attitudeAgeMs;
        doc["bridgeMountOffsetDeg"] = bs.mountOffsetMeasuredDeg;
        doc["bridgeMountSamples"] = bs.mountOffsetSamples;
        // The one number that AgOpenGPS and this firmware must both use, with AOG's
        // sign convention (body = received + value): the value actually used, and
        // what the live measurement says it should be (its negative). The web UI
        // shows both so the operator can type the same number into both places.
        doc["bridgeBaselineM"] = bs.baselineLengthM;
        doc["bridgeBaselineDevPct"] = bs.baselineDevPct;
        // Calibration readouts: the wheel angle scale the data suggests, and the
        // baseline length reference the F9P drift test uses (learned from the
        // receiver unless a mechanical value was entered).
        // Passive mode (2026-09-23): the estimators run and report, the model keeps
        // the configured values. The web page says so explicitly, otherwise a
        // learned value would look as if it were already in effect.
        // Geometry estimate (SCALE only since 2026-09-27, see [B8.103] - the offset term was
        // dropped as ill-conditioned; the zero belongs to the WAS/VWAS) and the
        // model-plausibility gate (2026-09-27). steerOffsetEst is DIAGNOSTIC/REPORTED only:
        // the model applies no offset of its own (sc_b stays 0).
        // Wheel angle reliability from the VWAS (0...100, -1 = not reported) and what the
        // bridge currently does with it (2026-09-28).
        // Direction of travel: the published heading stays the vehicle front axis,
        // the path based sources are mirrored by 180 deg while reversing.
        doc["bridgeReverse"] = bs.reverseDirection;
        // IMU yaw-rate support: whether the gyro currently drives psi, the rate it
        // delivers and the learned bias (a large bias = wrong sign/mounting).
        // Output smoothing state (active while the estimate is not trustworthy) and the
        // heading that actually goes to AOG.
        // Offset-corrected IMU heading (AgOpenGPS' imuGPS_Offset idea, learned from the
        // attitude) and the offset itself.
        // Antenna spacing: the value in use, and whether it is a FIXED reference (entered in
        // the web UI or the build default, Config::physicalBaselineDefaultM) or still learned -
        // plus whether the safety net has taken the test out because it rejects everything.
        // Active reference check (2026-10-11): the length the receiver solves on the first
        // clean frames (no reference involved) and whether that contradicts the FIXED
        // reference in use. The page shows both, so a mismatch is visible after ~4 s
        // instead of only after the safety net has muted the test (~30 s).
        // The dual-antenna heading offset in use (the same number AgOpenGPS carries). Shown
        // next to dualHeadingOffsetMeasuredDeg below, so a mismatch is visible at a glance.
        // The receiver's own per-epoch heading sigma (UNIHEADINGA field 6). The output
        // quality gate reads it, so it belongs next to the diagnostics it explains.
        // Side slip (path vs body) and the currently effective measurement noise,
        // so a field test can tell a tuning problem from a reception problem.
        doc["bridgeSideSlipDeg"] = headingBridge.getConfig().estimateSideSlip
                                   ? headingBridge.getSideSlipDeg() : 0.0f;
        // Forward projection: what the model predicted versus what the receiver
        // measured. ctk is the cross-track residual (= heading error, the quantity
        // the projection turns into a measurement), along the along-track residual
        // (= speed/scale error), rng the interval length it was derived over.
    }
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

// ---------------------------------------------------------------------------
// /api/bridge/geometry - the antenna geometry of the dual-antenna baseline
// ---------------------------------------------------------------------------
// TWO values, both of which the firmware AND AgOpenGPS have to agree on:
//   meters    - the mechanical antenna spacing, the reference of the baseline-length
//               plausibility test (HeadingBridge::Config::physicalBaselineM)
//   offsetDeg - the dual-antenna heading offset, the SAME number AgOpenGPS carries as its
//               "dual heading offset" (HeadingBridge::Config::dualHeadingOffsetDeg)
//
// Own endpoint instead of keys in /api/vwas/config because
//   * they are HeadingBridge settings, not VWAS ones, and
//   * handleVWASConfig() answers 503 without an active VWAS, which must not be able to block
//     a heading setting.
//
// GET -> everything needed to JUDGE the values: what is in use, whether the reference is
//        fixed or learned, the reference actually applied, the length the receiver solved,
//        the deviation between them, the mounting offset the firmware MEASURED itself, and
//        whether the safety net has taken the baseline test out.
// POST {"meters":<antenna spacing in m>} and/or {"offsetDeg":<angle in deg>} -> set + persist.
//        meters 0 means "switch the test off and learn again".
void FLASHMEM SimpleWebManager::handleBridgeGeometry(EthernetClient& client, const String& method) {
    if (method == "POST") {
        String body = readPostBody(client);
        StaticJsonDocument<256> in;
        DeserializationError error = deserializeJson(in, body);
        if (error) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }
        if (in["meters"].isNull() && in["offsetDeg"].isNull() && in["useMeasured"].isNull()) {
            SimpleHTTPServer::sendJSON(client,
                "{\"status\":\"error\",\"message\":\"meters or offsetDeg missing\"}");
            return;
        }
        // "use measured reference" (2026-10-11): adopt the length the receiver SOLVES as the
        // fixed reference. Deliberately an explicit operator action - the firmware measures
        // and REPORTS (Status::baselineSuggestionM) but never changes the reference itself,
        // because a wrong suggestion adopted automatically would be the very failure the
        // safety net exists for. Requires a completed measurement window.
        if (!in["useMeasured"].isNull() && in["useMeasured"].as<bool>()) {
            const HeadingBridge::Status& s = headingBridge.getStatus();
            if (s.baselineSuggestCount == 0 || !(s.baselineSuggestionM > 0.1f)) {
                SimpleHTTPServer::sendJSON(client,
                    "{\"status\":\"error\",\"message\":\"noch keine Messung - erst fahren, "
                    "bis saubere Frames gesammelt sind\"}");
                return;
            }
            headingBridge.setPhysicalBaselineM(s.baselineSuggestionM);
            const float eff = headingBridge.getPhysicalBaselineM();
            char respM[192];
            snprintf(respM, sizeof(respM),
                     "{\"status\":\"saved\",\"meters\":%.3f,\"offsetDeg\":%.2f,"
                     "\"used\":\"measured\",\"frames\":%u}",
                     (double)eff, (double)headingBridge.getDualHeadingOffsetDeg(),
                     (unsigned)s.baselineSuggestCount);
            SimpleHTTPServer::sendJSON(client, respM);
            return;
        }
        char resp[224];
        if (!in["meters"].isNull()) {
            const float want = in["meters"];
            // setPhysicalBaselineM() validates (0, or 0.1 ... 5.0 m), applies, persists and
            // logs. It reports nothing back, so compare the RESULT - that covers "cleared".
            headingBridge.setPhysicalBaselineM(want);
            const float eff = headingBridge.getPhysicalBaselineM();
            if (eff < want - 0.0005f || eff > want + 0.0005f) {
                snprintf(resp, sizeof(resp),
                         "{\"status\":\"error\",\"message\":\"%.3f m abgelehnt - erlaubt ist 0 "
                         "(aus) oder 0.1 ... 5.0\",\"meters\":%.3f}", (double)want, (double)eff);
                SimpleHTTPServer::sendJSON(client, resp);
                return;
            }
        }
        if (!in["offsetDeg"].isNull()) {
            const float want = in["offsetDeg"];
            headingBridge.setDualHeadingOffsetDeg(want);
            const float eff = headingBridge.getDualHeadingOffsetDeg();
            if (eff < want - 0.01f || eff > want + 0.01f) {
                snprintf(resp, sizeof(resp),
                         "{\"status\":\"error\",\"message\":\"%.2f deg abgelehnt - erlaubt ist "
                         "-180 ... 180\",\"offsetDeg\":%.2f}", (double)want, (double)eff);
                SimpleHTTPServer::sendJSON(client, resp);
                return;
            }
        }
        snprintf(resp, sizeof(resp),
                 "{\"status\":\"saved\",\"meters\":%.3f,\"offsetDeg\":%.2f}",
                 (double)headingBridge.getPhysicalBaselineM(),
                 (double)headingBridge.getDualHeadingOffsetDeg());
        SimpleHTTPServer::sendJSON(client, resp);
        return;
    }

    const HeadingBridge::Status& bs = headingBridge.getStatus();
    StaticJsonDocument<768> doc;
    doc["meters"]       = headingBridge.getPhysicalBaselineM();
    doc["set"]          = headingBridge.isPhysicalBaselineSet();
    doc["defaultM"]     = HeadingBridge::Config::physicalBaselineDefaultM;
    doc["offsetDeg"]    = headingBridge.getDualHeadingOffsetDeg();
    // getInstance() instead of the block-local `extern ConfigManager configManager;` the other
    // handlers use: that one is scoped to their function bodies and is not visible here.
    ConfigManager* cm = ConfigManager::getInstance();
    doc["offsetStored"] = cm ? cm->isBridgeDualHeadingOffsetValid() : false;
    doc["offsetDefaultDeg"] = HeadingBridge::Config::dualHeadingOffsetDefaultDeg;
    doc["refM"]         = bs.baselineRefM;
    doc["lenM"]         = bs.baselineLengthM;
    doc["devPct"]       = bs.baselineDevPct;
    doc["learnCount"]   = bs.baselineLearnCount;
    doc["testDisabled"] = bs.baselineTestDisabled;
    // Active reference check (2026-10-11): what the receiver SOLVES on the first clean
    // frames, with no reference involved - shown next to the reference in use, and
    // adoptable with one click ("use measured reference").
    doc["suggestM"]      = bs.baselineSuggestionM;
    doc["suggestCount"]  = bs.baselineSuggestCount;
    doc["suggestFrames"] = headingBridge.getConfig().baselineSuggestFrames;
    doc["refSuspect"]    = bs.baselineRefSuspect;
    doc["hdgSdDeg"]     = gnssProcessor.getData().uniHdgStdDevDeg;
    // What the firmware measures itself (azimuth - direction of travel). Its NEGATIVE is the
    // value for dualHeadingOffsetDeg, so the page can compare the two and say "passt".
    doc["mountMeasDeg"] = bs.mountOffsetMeasuredDeg;
    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

void FLASHMEM SimpleWebManager::handleVWASConfig(EthernetClient& client, const String& method) {
    if (!wheelAngleFusionPtr) {
        SimpleHTTPServer::send(client, 503, "application/json",
                                "{\"error\":\"VWAS not active\"}");
        return;
    }

    if (method == "POST") {
        String body = readPostBody(client);
        StaticJsonDocument<768> doc;
        DeserializationError error = deserializeJson(doc, body);
        if (error) {
            SimpleHTTPServer::sendJSON(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        WheelAngleFusion::Config cfg = wheelAngleFusionPtr->getConfig();
        if (!doc["wheelbase"].isNull()) cfg.wheelbase = doc["wheelbase"];
        if (!doc["maxSteeringAngle"].isNull()) cfg.maxSteeringAngle = doc["maxSteeringAngle"];
        if (!doc["vehicleType"].isNull()) cfg.vehicleType = doc["vehicleType"];
        // B10.52: die Antennen-Position gehoert der BRIDGE (EEPROM 1600-1699), nicht dem
        // VWAS-Blob - sie darf dessen Layout und die Endanschlag-Kalibrierung nicht anfassen.
        if (!doc["antennaPivotM"].isNull()) {
            ConfigManager* cmPiv = ConfigManager::getInstance();
            if (cmPiv) { cmPiv->setBridgeAntennaPivotM((float)doc["antennaPivotM"], true); }
        }

        // A manual countsPerDegree entry counts as calibrated - the Config
        // struct documents exactly that contract ("set to true by a successful
        // endstop calibration OR by an explicit manual cpd entry"). Without
        // this, the boot "NOT CALIBRATED" warning and the web UI banner would
        // persist forever for an operator who measures cpd by hand instead of
        // using the three-button endstop calibration.
        // Range-checked against the same window the endstop calibration uses,
        // so a typo cannot mark the config "calibrated" with a nonsense value.
        if (!doc["countsPerDegree"].isNull()) {
            float cpd = doc["countsPerDegree"];
            if (cpd >= WheelAngleFusion::CPD_MIN && cpd <= WheelAngleFusion::CPD_MAX) {
                cfg.countsPerDegree = cpd;
                cfg.cpdCalibrated = true;
            } else {
                LOG_WARNING(EventSource::NETWORK,
                            "VWAS config: countsPerDegree %.2f outside [%.0f..%.0f] rejected",
                            cpd, WheelAngleFusion::CPD_MIN, WheelAngleFusion::CPD_MAX);
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg),
                             "countsPerDegree %.1f outside %.0f..%.0f - nothing saved",
                             cpd, WheelAngleFusion::CPD_MIN, WheelAngleFusion::CPD_MAX);
                    char resp[192];
                    snprintf(resp, sizeof(resp),
                             "{\"status\":\"error\",\"message\":\"%s\"}", msg);
                    SimpleHTTPServer::sendJSON(client, resp);
                }
                return;
            }
        }

        // Sidehill gain (P2). MAGNITUDE in AgOpenGPS's convention - positive, the
        // same number as his "Side Hill Draft Gain" slider (0.12..0.15 on his
        // machine). The SIGN is derived at runtime from the invert state
        // (WheelAngleFusion::effectiveSideHillGain()), because a positive slider on
        // an inverted reported angle has to become a negative internal gain.
        //
        // No range check here on purpose: the runtime owns the plausible window, it
        // clamps to it and logs a warning, so there is exactly one place that decides
        // the limit. The effective value comes back in the response below, so the UI
        // can show what was actually applied instead of what was typed.
        if (!doc["sideHillGain"].isNull()) {
            float g = doc["sideHillGain"];
            if (g < 0.0f) g = -g;              // magnitude only
            cfg.sideHillGainDegPerDeg = g;
        }
        if (!doc["sideHillGainSignFlip"].isNull()) {
        }

        wheelAngleFusionPtr->setConfig(cfg);
        AutosteerProcessor::getInstance()->saveVehicleSetup();

        LOG_INFO(EventSource::NETWORK, "VWAS config updated via web UI");
        {
            // Report the DERIVED value, not the entered one: that is the number the
            // residual is computed with, and it carries the sign.
            char resp[160];
            snprintf(resp, sizeof(resp),
                     "{\"status\":\"saved\",\"sideHillGainEffective\":%.3f}",
                     wheelAngleFusionPtr->effectiveSideHillGain());
            SimpleHTTPServer::sendJSON(client, resp);
        }
        return;
    }

    // GET - return current config
    const WheelAngleFusion::Config& cfg = wheelAngleFusionPtr->getConfig();
    StaticJsonDocument<768> doc;
    doc["wheelbase"] = cfg.wheelbase;
    doc["maxSteeringAngle"] = cfg.maxSteeringAngle;
    doc["vehicleType"] = cfg.vehicleType;
    {
        ConfigManager* cmPiv = ConfigManager::getInstance();
        doc["antennaPivotM"] = cmPiv ? cmPiv->getBridgeAntennaPivotM() : 0.0f;
    }
    doc["preferDualHeading"] = cfg.preferDualHeading;
    doc["countsPerDegree"] = cfg.countsPerDegree;
    doc["invertEncoder"] = cfg.invertEncoder;
    // Sidehill gain (P2): the configured MAGNITUDE in AgOpenGPS's convention, the
    // derived value that is actually used, and what the estimator reports. The UI
    // shows all three - a reported value far from the configured one, or of the
    // opposite sign, is exactly the fault that has to be visible without a rebuild.
    doc["sideHillGain"] = cfg.sideHillGainDegPerDeg;
    doc["sideHillGainSignFlip"] = cfg.sideHillGainSignFlip;
    doc["sideHillGainEffective"] = wheelAngleFusionPtr->effectiveSideHillGain();
    doc["sideHillGainReported"] = wheelAngleFusionPtr->getReportedSideHillGain();

    String json;
    serializeJson(doc, json);
    SimpleHTTPServer::sendJSON(client, json);
}

void FLASHMEM SimpleWebManager::handleHeadingDump(EthernetClient& client) {
    headingMonitor.requestDump("web request");

    char json[192];
    snprintf(json, sizeof(json),
             "{\"status\":\"ok\",\"samples\":%lu,\"events\":%lu,\"eventActive\":%d,"
             "\"lastEventMs\":%lu,\"lastGapMs\":%lu,\"dumpPending\":%d}",
             (unsigned long)headingMonitor.getSamplesTaken(),
             (unsigned long)headingMonitor.getEventCount(),
             headingMonitor.isEventActive() ? 1 : 0,
             (unsigned long)headingMonitor.getLastEventDurationMs(),
             (unsigned long)headingMonitor.getLastGapMs(),
             headingMonitor.isDumpPending() ? 1 : 0);
    SimpleHTTPServer::sendJSON(client, json);
}







