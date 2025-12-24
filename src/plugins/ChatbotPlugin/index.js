const path = require('path');

module.exports = function plugin(context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './client')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@site/src/components/ChatbotWidget': path.resolve(
              __dirname,
              '../../components/ChatbotWidget'
            ),
          },
        },
      };
    },
  };
};