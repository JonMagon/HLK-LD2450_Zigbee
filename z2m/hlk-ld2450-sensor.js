import * as exposes from "zigbee-herdsman-converters/lib/exposes";
import * as m from "zigbee-herdsman-converters/lib/modernExtend";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";

const CUSTOM_CLUSTER_ID = 0xFC00;
const CUSTOM_ATTRIBUTE_ID = 0x0001;
const ENDPOINT = 10;

const LD2450_VERSION_ATTR_ID = 0x04;

export default {
    zigbeeModel: ['esp32h2'],
    model: 'CUSTOM_DEVICE',
    vendor: 'JonMagon',
    description: 'Test device',
    
    endpoint: (device) => {
        return {custom: ENDPOINT};
    },

    
    extend: [
        m.deviceAddCustomCluster("customCluster", {
            ID: 0xFC00,
            attributes: {
                customByte: {ID: 0x0001, type: 0x20},
                firmwareVersion: {ID: LD2450_VERSION_ATTR_ID, type: 0x42},
            },
            commands: {},
            commandsResponse: {},
        }),
        
        m.numeric({
            name: "custom_byte",
            cluster: "customCluster",
            attribute: "customByte",
            description: "LED Color test byte",
            valueMin: 0,
            valueMax: 255,
            endpoint: "custom",
        }),

        m.text({
            name: "LD2450_firmware_version",
            cluster: "customCluster",
            attribute: "firmwareVersion",
            description: "LD2450 Radar Firmware Version",
            endpoint: "custom",
            //access: 'STATE',
        }),
    ],

    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(ENDPOINT);
        if (!endpoint) {
            throw new Error('Endpoint does not exist.');
        }
        
        await reporting.bind(endpoint, coordinatorEndpoint, ["customCluster"]);

        await endpoint.configureReporting("customCluster", [
            {
                attribute: "customByte",
                minimumReportInterval: 1,
                maximumReportInterval: 3600,
                reportableChange: 1
            },
            {
                attribute: "firmwareVersion",
                minimumReportInterval: 1,
                maximumReportInterval: 86400,
                reportableChange: 0
            }
        ]);
    },
};
