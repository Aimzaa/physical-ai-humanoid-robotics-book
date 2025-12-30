// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to type-check this file
// even if they don't support TypeScript syntax yet.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics — Embodied Intelligence in the Real World',
  tagline: 'A comprehensive guide to Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://aimzaa.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually /<projectName>/
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'Aimzaa', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Aimzaa/physical-ai-humanoid-robotics-book/tree/main/',
        },
        blog: false, // Disable blog plugin as we're creating a book, not a blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    './src/plugins/ChatbotPlugin',
    './src/plugins/AuthPlugin',
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics Book',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Book',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'book',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/Aimzaa/physical-ai-humanoid-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'custom-authButton',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/docs/',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module-1-robotic-nervous-system/chapter-1-1-introduction-ros2',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone-project/autonomous-humanoid-implementation',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'References',
                to: '/docs/references',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/Aimzaa/physical-ai-humanoid-robotics-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;