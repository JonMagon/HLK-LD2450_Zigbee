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

                target_1_x: {ID: 0x0005, type: 0x29},
                target_1_y: {ID: 0x0006, type: 0x29},
                target_1_speed: {ID: 0x0007, type: 0x29},
                target_1_distance: {ID: 0x0008, type: 0x21},
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

        m.numeric({
            name: "target_1_x",
            cluster: "customCluster",
            attribute: "target_1_x",
            description: "target_1_x",
            endpoint: "custom",
            access: 'STATE',
        }),

        m.numeric({
            name: "target_1_y",
            cluster: "customCluster",
            attribute: "target_1_y",
            description: "target_1_y",
            endpoint: "custom",
            access: 'STATE',
        }),

        m.numeric({
            name: "target_1_speed",
            cluster: "customCluster",
            attribute: "target_1_speed",
            description: "target_1_speed",
            endpoint: "custom",
            access: 'STATE',
        }),

        m.numeric({
            name: "target_1_distance",
            cluster: "customCluster",
            attribute: "target_1_distance",
            description: "target_1_distance",
            endpoint: "custom",
            access: 'STATE',
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

    fromZigbee: [
        {
            cluster: 'customCluster',
            type: 'write',
            convert: (model, msg, publish, options, meta) => {
                const result = {};

                if (msg.data.hasOwnProperty('target_1_x')) {
                    result.target_1_x = msg.data.target_1_x;
                }
                if (msg.data.hasOwnProperty('target_1_y')) {
                    result.target_1_y = msg.data.target_1_y;
                }

                if (msg.data.hasOwnProperty('target_1_x') && msg.data.hasOwnProperty('target_1_y')) {
                    result.target_1_distance = Math.round(
                        Math.sqrt(Math.pow(msg.data.target_1_y, 2) + Math.pow(msg.data.target_1_y, 2))
                    )
                }

                if (msg.data.hasOwnProperty('target_1_speed')) {
                    result.target_1_speed = msg.data.target_1_speed;
                }
                if (msg.data.hasOwnProperty('target_1_distance')) {
                    result.target_1_distance = msg.data.target_1_distance;
                }
                if (msg.data.hasOwnProperty('customByte')) {
                    result.custom_byte = msg.data.customByte;
                }
                if (msg.data.hasOwnProperty('firmwareVersion')) {
                    result.LD2450_firmware_version = msg.data.firmwareVersion;
                }

                return result;
            },
        },
    ],
};
