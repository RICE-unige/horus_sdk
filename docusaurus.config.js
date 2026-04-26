// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "HORUS SDK",
  tagline: "Robot registration and mixed-reality integration for ROS 2 fleets",
  favicon: "img/favicon.svg",
  url: "https://rice-unige.github.io",
  baseUrl: "/horus_sdk/",
  organizationName: "RICE-unige",
  projectName: "horus_sdk",
  trailingSlash: false,
  onBrokenLinks: "throw",
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: "throw"
    }
  },
  i18n: {
    defaultLocale: "en",
    locales: ["en"]
  },
  presets: [
    [
      "classic",
      {
        docs: {
          routeBasePath: "docs",
          sidebarPath: require.resolve("./sidebars.js"),
          editUrl: "https://github.com/RICE-unige/horus_sdk/tree/docs/site-v1/"
        },
        blog: false,
        theme: {
          customCss: require.resolve("./src/css/custom.css")
        }
      }
    ]
  ],
  themeConfig: /** @type {import('@docusaurus/preset-classic').ThemeConfig} */ ({
    image: "img/horus_logo_black.svg",
    navbar: {
      title: "HORUS SDK",
      logo: {
        alt: "Horus SDK Logo",
        src: "img/horus_logo_black.svg",
        srcDark: "img/horus_log_white.svg"
      },
      items: [
        { to: "/docs/intro", label: "Overview", position: "left" },
        { to: "/docs/getting-started/quickstart", label: "Quickstart", position: "left" },
        { to: "/docs/examples/registration-flows", label: "Examples", position: "left" },
        { to: "/docs/python-sdk/registration", label: "Python SDK", position: "left" },
        { to: "/docs/integration/horus-ros2", label: "Integration", position: "left" },
        {
          href: "https://github.com/RICE-unige/horus_sdk",
          label: "GitHub",
          position: "right"
        }
      ]
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Getting started",
          items: [
            { label: "Overview", to: "/docs/intro" },
            { label: "Installation", to: "/docs/getting-started/installation" },
            { label: "Quickstart", to: "/docs/getting-started/quickstart" }
          ]
        },
        {
          title: "Core guides",
          items: [
            { label: "Robot model", to: "/docs/python-sdk/robot-model" },
            { label: "Sensors", to: "/docs/python-sdk/sensors" },
            { label: "DataViz", to: "/docs/python-sdk/dataviz" }
          ]
        },
        {
          title: "Workflows",
          items: [
            { label: "Curated examples", to: "/docs/examples/registration-flows" },
            { label: "Map workflows", to: "/docs/examples/occupancy-grid-workflow" },
            { label: "Live robot workflows", to: "/docs/examples/topic-monitoring-dashboard" }
          ]
        }
      ],
      copyright: `Copyright ${new Date().getFullYear()} RICE Lab, University of Genoa. Built with Docusaurus.`
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: ["bash", "cpp", "rust", "json", "yaml"]
    },
    colorMode: {
      defaultMode: "light",
      disableSwitch: false,
      respectPrefersColorScheme: true
    }
  })
};

module.exports = config;
