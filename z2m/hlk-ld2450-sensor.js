import * as exposes from "zigbee-herdsman-converters/lib/exposes";
import * as m from "zigbee-herdsman-converters/lib/modernExtend";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";

const CUSTOM_CLUSTER_ID = 0xFC00;
const CUSTOM_ATTRIBUTE_ID = 0x0001;
const ENDPOINT = 10;

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
            }
        ]);
    },
};
