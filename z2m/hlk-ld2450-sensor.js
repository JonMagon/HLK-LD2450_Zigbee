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
                target_1_speed: {ID: 0x0007, type: 0x29}
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
        }),
    ],

    exposes: [
        exposes.composite('target_1', 'target_1', exposes.access.STATE)
            .withDescription('Target 1 tracking data')
            .withFeature(exposes.numeric('x', exposes.access.STATE).withUnit('mm').withDescription('X coordinate'))
            .withFeature(exposes.numeric('y', exposes.access.STATE).withUnit('mm').withDescription('Y coordinate'))
            .withFeature(exposes.numeric('speed', exposes.access.STATE).withUnit('mm/s').withDescription('Speed'))
            .withFeature(exposes.numeric('distance', exposes.access.STATE).withUnit('mm').withDescription('Distance'))
            .withFeature(exposes.numeric('angle', exposes.access.STATE).withUnit('°').withDescription('Angle'))
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

                const target1Data = {};

                if (msg.data.hasOwnProperty('target_1_x')) {
                    target1Data.x = msg.data.target_1_x;
                }
                if (msg.data.hasOwnProperty('target_1_y')) {
                    target1Data.y = msg.data.target_1_y;
                }
                if (msg.data.hasOwnProperty('target_1_speed')) {
                    target1Data.speed = msg.data.target_1_speed;
                }

                if (msg.data.hasOwnProperty('target_1_x') && msg.data.hasOwnProperty('target_1_y')) {
                    target1Data.distance = Math.round(
                        Math.sqrt(Math.pow(msg.data.target_1_x, 2) + Math.pow(msg.data.target_1_y, 2))
                    );

                    const angleDeg = Math.atan2(msg.data.target_1_x, msg.data.target_1_y) * (180 / Math.PI);
                    target1Data.angle = Math.round(angleDeg * 10) / 10;
                }

                result.target_1 = target1Data;


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