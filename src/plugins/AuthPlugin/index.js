// Auth Plugin for Docusaurus - simplified version
// AuthButton is now added via navbar config using theme swizzling

module.exports = function authPlugin(context, options) {
  return {
    name: 'docusaurus-auth-plugin',
    // No client modules needed - we use theme swizzling instead
  };
};
