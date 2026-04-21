/*
  Zigbee Lidar Distance Sensor Driver

  Author:  Designed by John Land, but written by Claude (AI) & ChatGPT
  Version: 1.28
  Date:    2026-04-08

  Description: Driver for ESP32-C6 Zigbee connected to VL53L1X Lidar Distance Sensor
  Distances reported in millimeters, centimeters, meters, inches, feet.

  NOTE: uses the Zigbee Illuminance cluster to report distance & presence.

  Dedicated to the public domain in 2025

  Unless required by applicable law or agreed to in writing, this software is distributed
  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

metadata {
    definition (name: "Zigbee Lidar Distance Sensor", namespace: "John Land v. 1.28", author: "John Land") {
        capability "Sensor"
        capability "IlluminanceMeasurement"  // For Zigbee compatibility
        capability "PresenceSensor"          // For occupancy detection

        // Custom attributes
        // Smoothed distance attributes (endpoint 10)
        attribute "Smoothed Millimeters", "number"
        attribute "Selected Display", "number"
        attribute "Smoothed Summary", "string"

        // Raw (unsmoothed) distance attributes (endpoint 11)
        attribute "Raw Millimeters", "number"
        attribute "Raw Summary", "string"

        // Device info
        attribute "firmwareVersion", "string"

        // Health monitoring
        attribute "healthStatus", "enum", ["online", "offline"]
        attribute "lastCheckin", "string"

        // WiFi mode control
        command "enableWiFiMode"

        // Commands
        command "refresh"
        command "setPresenceThreshold", [[name:"Threshold (mm)", type: "NUMBER", description: "Distance threshold in mm for presence detection"]]

        fingerprint profileId: "0104", inClusters: "0000,0003,0400,0006", outClusters: "", manufacturer: "Espressif", deviceJoinName: "Zigbee Lidar Distance Sensor"
    }

    preferences {
        input name: "unitPreference", type: "enum", title: "Display Units",
            options: ["mm": "Millimeters", "cm": "Centimeters", "m": "Meters", "in": "Inches", "ft": "Feet", "yd": "Yards"],
            defaultValue: "mm", required: true
        input name: "presenceThreshold", type: "number", title: "Presence Detection Threshold (mm)",
            description: "Distance below this value triggers 'present' status",
            defaultValue: 1500, range: "0..65535"
        input name: "minReportInterval", type: "number", title: "Minimum Report Interval (seconds)",
            defaultValue: 10, range: "0..3600"
        input name: "maxReportInterval", type: "number", title: "Maximum Report Interval (seconds)",
            defaultValue: 300, range: "1..3600"
        input name: "reportDelta", type: "number", title: "Report Delta (mm)",
            description: "Minimum change to trigger report",
            defaultValue: 50, range: "1..1000"
        input name: "healthTimeoutMinutes", type: "number", title: "Health Timeout (minutes)",
            description: "Mark device offline if no Zigbee messages are received within this many minutes",
            defaultValue: 30, range: "1..1440"
        input name: "logEnable", type: "bool", title: "Enable debug logging", defaultValue: true
    }
}

def installed() {
    log.info "Zigbee Lidar Distance Sensor installed"
    sendEvent(name: "presence", value: "not present")
    sendEvent(name: "Selected Display", value: 0, unit: getUnitLabel())
    sendEvent(name: "healthStatus", value: "offline")

    state.lastSeen = 0L
    state.lastCheckinEventMs = 0L

    scheduleHealthCheck()
    configure()  // configure() reads 0x0005; parse() will update firmwareVersion
}

def updated() {
    log.info "Zigbee Lidar Distance Sensor updated"
    if (logEnable) runIn(1800, logsOff)

    if (device.currentValue("healthStatus") == null) sendEvent(name: "healthStatus", value: "offline")
    scheduleHealthCheck()
    configure()  // configure() reads 0x0005; parse() will update firmwareVersion
}

def configure() {
    log.info "Configuring Zigbee Lidar Distance Sensor..."
    def cmds = []
    cmds += zigbee.readAttribute(0x0000, 0x0005, [destEndpoint: 10])  // Basic cluster model string (ep10 has setManufacturerAndModel)

    // Endpoint 10: smoothed distance
    cmds += zigbee.configureReporting(0x0400, 0x0000, 0x21,
        (minReportInterval ?: 5).toInteger(),
        (maxReportInterval ?: 300).toInteger(),
        (reportDelta ?: 10).toInteger(),
        [destEndpoint: 10])
    cmds += zigbee.readAttribute(0x0400, 0x0000, [destEndpoint: 10])
    cmds += zigbee.readAttribute(0x0400, 0x0001, [destEndpoint: 10])
    cmds += zigbee.readAttribute(0x0400, 0x0002, [destEndpoint: 10])

    // Endpoint 11: raw (unsmoothed) distance
    cmds += zigbee.configureReporting(0x0400, 0x0000, 0x21,
        (minReportInterval ?: 5).toInteger(),
        (maxReportInterval ?: 300).toInteger(),
        1,                                         // Raw endpoint: report every 1mm change
        [destEndpoint: 11])
    cmds += zigbee.readAttribute(0x0400, 0x0000, [destEndpoint: 11])

    // Endpoint 12: WiFi mode switch (read current on/off state)
    cmds += zigbee.readAttribute(0x0006, 0x0000, [destEndpoint: 12])

    return cmds
}

def refresh() {
    if (logEnable) log.debug "Refreshing distance reading..."
    def cmds = []
    cmds += zigbee.readAttribute(0x0400, 0x0000)
    return cmds
}

def parse(String description) {
    if (logEnable) log.debug "Parsing: ${description}"
    markOnline()

    // Parse endpoint first — zigbee.getEvent() does not carry endpoint info reliably
    // so we always use descMap to determine which endpoint fired before acting.
    def descMap = zigbee.parseDescriptionAsMap(description)
    def ep = descMap?.endpoint ? Integer.parseInt(descMap.endpoint, 16) : 0

    // Handle On/Off cluster reports by endpoint
    if (descMap?.clusterInt == 0x0006 && descMap?.attrInt == 0x0000) {
        def val = descMap.value == "01" ? "on" : "off"
        if (ep == 12) {
            // WiFi mode switch — transient; ignore incoming state (on() handles it)
            if (logEnable) log.debug "WiFi switch state from ep12: ${val} (ignored)"
            return
        }
    }

    // Handle all known cluster/attribute combinations explicitly BEFORE calling
    // zigbee.getEvent(), which may return early for unknown attribute types
    // and prevent our handlers from running.
    if (descMap?.cluster == "0000" && descMap?.attrId == "0005") {
        // Basic cluster model string — hex-encoded with leading length byte
        // e.g. "1C4C6964617244697374616E636553656E736F725F76312E36204F5441"
        //   0x1C = 28 chars, then ASCII for "LidarDistanceSensor_v1.6 OTA"
        def hexVal = descMap.value ?: ""
        def modelString = ""
        try {
            def hexData = hexVal.length() > 2 ? hexVal.substring(2) : hexVal
            for (int i = 0; i + 1 < hexData.length(); i += 2) {
                modelString += (char) Integer.parseInt(hexData.substring(i, i + 2), 16)
            }
        } catch (Exception e) {
            modelString = hexVal
        }
        if (logEnable) log.debug "Model string decoded: ${modelString}"
        def fwVersion = modelString?.contains("_v") ? modelString.split("_v")[1] : (modelString ?: "unknown")
        sendEventIfChanged("firmwareVersion", fwVersion, null, true)
        return
    } else if (descMap?.cluster == "0400" && descMap?.attrId == "0000") {
        def distanceMillimeters = Integer.parseInt(descMap.value, 16)
        ep = descMap.endpoint ? Integer.parseInt(descMap.endpoint, 16) : 10
        if (ep == 11) {
            if (logEnable) log.debug "Raw distance from ep11: ${distanceMillimeters} mm"
            processRawDistance(distanceMillimeters)
        } else {
            if (logEnable) log.debug "Smoothed distance from ep10: ${distanceMillimeters} mm"
            processDistance(distanceMillimeters)
        }
        return
    } else if (descMap?.cluster == "0400" && descMap?.attrId == "0001") {
        def minValue = Integer.parseInt(descMap.value, 16)
        if (logEnable) log.debug "Min distance: ${minValue} mm"
        return
    } else if (descMap?.cluster == "0400" && descMap?.attrId == "0002") {
        def maxValue = Integer.parseInt(descMap.value, 16)
        if (logEnable) log.debug "Max distance: ${maxValue} mm"
        return
    }

    // Explicitly suppress any remaining On/Off cluster messages so they never
    // reach getEvent() and create a spurious "switch" attribute in Current States.
    if (descMap?.cluster == "0006") {
        if (logEnable) log.debug "On/Off cluster message handled or suppressed (ep${ep})"
        return
    }

    // Fall through to getEvent() for anything else (presence, illuminance, etc.)
    // Explicitly exclude "switch" so getEvent() can never create that attribute.
    def result = zigbee.getEvent(description)
    if (result && result.name != "switch") return result
}

def processDistance(distanceMillimeters) {
    def values = getDistanceValues(distanceMillimeters)

    // Store smoothed distance formats
    sendEventIfChanged("Smoothed Millimeters", values.mm, "mm")
    sendEventIfChanged("Smoothed Summary", buildDistanceSummary(values), null, false)

    // Send primary distance event with preferred units
    def displayValue = getPreferredDisplayValue(values)
    def displayUnit = getUnitLabel()

    sendEventIfChanged("Selected Display", displayValue, displayUnit)

    // Check presence threshold
    def threshold = presenceThreshold ?: 1500
    def newPresence = (distanceMillimeters < threshold) ? "present" : "not present"
    log.info "Distance: ${distanceMillimeters}mm (${displayValue} ${displayUnit})"
    if (device.currentValue("presence") != newPresence) {
        sendEvent(name: "presence", value: newPresence)
        log.info "Presence changed to: ${newPresence} (distance: ${distanceMillimeters}mm, threshold: ${threshold}mm)"
    }
}

def processRawDistance(distanceMillimeters) {
    def values = getDistanceValues(distanceMillimeters)

    sendEventIfChanged("Raw Millimeters", values.mm, "mm")
    sendEventIfChanged("Raw Summary", buildDistanceSummary(values), null, false)

    if (logEnable) {
        log.debug "Raw distance stored: ${distanceMillimeters} mm"
    }
}

def enableWiFiMode() {
    log.info "Sending WiFi-only mode request to device (endpoint 12 On command)..."
    return zigbee.command(0x0006, 0x01, [destEndpoint: 12])          //On command to endpoint 12
}

def setPresenceThreshold(threshold) {
    if (logEnable) log.debug "Setting presence threshold to ${threshold}mm"
    device.updateSetting("presenceThreshold", [value: threshold, type: "number"])
    // Re-evaluate presence with new threshold
    if (device.currentValue("Smoothed Millimeters")) {
        processDistance(device.currentValue("Smoothed Millimeters").toInteger())
    }
}

def scheduleHealthCheck() {
    unschedule("deviceHealthCheck")
    runEvery30Minutes("deviceHealthCheck")
}

def markOnline() {
    Long nowMs = now()
    state.lastSeen = nowMs

    Long lastCheckinEventMs = (state.lastCheckinEventMs ?: 0L) as Long
    if (lastCheckinEventMs == 0L || (nowMs - lastCheckinEventMs) >= 60000L || device.currentValue("lastCheckin") == null) {
        def tz = location?.timeZone ?: TimeZone.getTimeZone("UTC")
        def ts = new Date(nowMs).format("yyyy-MM-dd HH:mm:ss", tz)
        sendEventIfChanged("lastCheckin", ts, null, false)
        state.lastCheckinEventMs = nowMs
    }

    if (device.currentValue("healthStatus") != "online") {
        sendEvent(name: "healthStatus", value: "online")
        log.info "Device healthStatus changed to online"
    }
}

def deviceHealthCheck() {
    Long lastSeen = (state.lastSeen ?: 0L) as Long
    Integer timeoutMinutes = (settings.healthTimeoutMinutes ?: 30) as Integer
    Long timeoutMs = timeoutMinutes * 60L * 1000L

    if (lastSeen == 0L || (now() - lastSeen) >= timeoutMs) {
        if (device.currentValue("healthStatus") != "offline") {
            sendEvent(name: "healthStatus", value: "offline")
            log.warn "Device healthStatus changed to offline"
        }
    } else if (logEnable) {
        log.debug "Health check OK - last Zigbee message received ${Math.round((now() - lastSeen) / 60000.0)} minute(s) ago"
    }
}

private Map getDistanceValues(Integer distanceMillimeters) {
    return [
        mm    : distanceMillimeters,
        cm    : roundValue(distanceMillimeters / 10.0,   1),
        m     : roundValue(distanceMillimeters / 1000.0, 3),
        inches: roundValue(distanceMillimeters / 25.4,   2),
        feet  : roundValue(distanceMillimeters / 304.8,  2),
        yards : roundValue(distanceMillimeters / 914.4,  2)
    ]
}

private getPreferredDisplayValue(Map values) {
    switch(unitPreference) {
        case "cm": return values.cm
        case "m" : return values.m
        case "in": return values.inches
        case "ft": return values.feet
        case "yd": return values.yards
        default  : return values.mm
    }
}

private String buildDistanceSummary(Map values) {
    return "${values.mm} mm | ${values.cm} cm | ${values.m} m | ${values.inches} in | ${values.feet} ft | ${values.yards} yd"
}

private BigDecimal roundValue(value, Integer places) {
    return new BigDecimal(value.toString()).setScale(places, BigDecimal.ROUND_HALF_UP)
}

private void sendEventIfChanged(String name, value, String unit = null, Boolean displayed = false) {
    def current = device.currentValue(name)
    String newValue = value?.toString()
    String currentValue = current?.toString()

    if (currentValue != newValue) {
        def evt = [name: name, value: value, displayed: displayed]
        if (unit) evt.unit = unit
        sendEvent(evt)
    }
}

def getUnitLabel() {
    switch(unitPreference) {
        case "cm": return "centimeters"
        case "m": return "meters"
        case "in": return "inches"
        case "ft": return "feet"
        case "yd": return "yards"
        default: return "millimeters"
    }
}

def logsOff() {
    log.warn "Debug logging disabled..."
    device.updateSetting("logEnable", [value: "false", type: "bool"])
}
