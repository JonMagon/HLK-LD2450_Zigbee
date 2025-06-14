import * as exposes from "zigbee-herdsman-converters/lib/exposes";
import * as m from "zigbee-herdsman-converters/lib/modernExtend";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";

const CUSTOM_CLUSTER_ID = 0xFC00;
const CUSTOM_ATTRIBUTE_ID = 0x0001;
const ENDPOINT = 10;

const LD2450_VERSION_ATTR_ID = 0x04;
const INVALID_COORDINATE_VALUE = 0x7FFF;

const TARGET_BASE_ATTR_ID = 0x0005;
const TARGETS_COUNT = 3;

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

                ...((() => {
                    const attrs = {};
                    for (let i = 1; i <= TARGETS_COUNT; i++) {
                        const baseId = TARGET_BASE_ATTR_ID + (i - 1) * 3;
                        attrs[`target_${i}_x`] = {ID: baseId, type: 0x29};
                        attrs[`target_${i}_y`] = {ID: baseId + 1, type: 0x29};
                        attrs[`target_${i}_speed`] = {ID: baseId + 2, type: 0x29};
                    }
                    return attrs;
                })())
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
        (() => {
            let trackingComposite = exposes.composite('tracking', 'tracking', exposes.access.STATE)
                .withDescription('Radar tracking system');

            for (let i = 1; i <= TARGETS_COUNT; i++) {
                trackingComposite = trackingComposite.withFeature(
                    exposes.composite(`target_${i}`, `target_${i}`, exposes.access.STATE)
                        .withDescription(`Target ${i} tracking data`)
                        .withFeature(exposes.numeric('x', exposes.access.STATE).withUnit('mm').withDescription('X coordinate'))
                        .withFeature(exposes.numeric('y', exposes.access.STATE).withUnit('mm').withDescription('Y coordinate'))
                        .withFeature(exposes.numeric('speed', exposes.access.STATE).withUnit('mm/s').withDescription('Speed'))
                        .withFeature(exposes.numeric('distance', exposes.access.STATE).withUnit('mm').withDescription('Distance'))
                        .withFeature(exposes.numeric('angle', exposes.access.STATE).withUnit('°').withDescription('Angle'))
                );
            }

            return trackingComposite;
        })()
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

                const processTargetData = (targetNum) => {
                    const targetData = {};
                    const xKey = `target_${targetNum}_x`;
                    const yKey = `target_${targetNum}_y`;
                    const speedKey = `target_${targetNum}_speed`;

                    if (msg.data.hasOwnProperty(xKey)) {
                        targetData.x = msg.data[xKey] === INVALID_COORDINATE_VALUE ? undefined : msg.data[xKey];
                    }
                    if (msg.data.hasOwnProperty(yKey)) {
                        targetData.y = msg.data[yKey] === INVALID_COORDINATE_VALUE ? undefined : msg.data[yKey];
                    }
                    if (msg.data.hasOwnProperty(speedKey)) {
                        targetData.speed = msg.data[speedKey] === INVALID_COORDINATE_VALUE ? undefined : msg.data[speedKey];
                    }

                    if (msg.data.hasOwnProperty(xKey) && msg.data.hasOwnProperty(yKey)) {
                        if (msg.data[xKey] === INVALID_COORDINATE_VALUE || msg.data[yKey] === INVALID_COORDINATE_VALUE) {
                            targetData.distance = undefined;
                            targetData.angle = undefined;
                        } else {
                            targetData.distance = Math.round(
                                Math.sqrt(Math.pow(msg.data[xKey], 2) + Math.pow(msg.data[yKey], 2))
                            );

                            const angleDeg = Math.atan2(msg.data[xKey], msg.data[yKey]) * (180 / Math.PI);
                            targetData.angle = Math.round(angleDeg * 10) / 10;
                        }
                    }

                    return targetData;
                };

                const trackingData = {};

                for (let i = 1; i <= TARGETS_COUNT; i++) {
                    const targetData = processTargetData(i);
                    if (Object.keys(targetData).length > 0) {
                        trackingData[`target_${i}`] = targetData;
                    }
                }

                if (Object.keys(trackingData).length > 0) {
                    result.tracking = trackingData;
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