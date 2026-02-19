// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "HORUS SDK",
  tagline: "Holistic Operational Reality for Unified Systems",
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
      title: "",
      logo: {
        alt: "Horus SDK Logo",
        src: "img/horus_logo_black.svg",
        srcDark: "img/horus_log_white.svg"
      },
      items: [
        { to: "/docs/intro", label: "Docs", position: "left" },
        { to: "/docs/getting-started/quickstart", label: "Quickstart", position: "left" },
        {
          href: "https://github.com/RICE-unige/horus_sdk",
          label: "GitHub",
          position: "right"
        }
      ]
    },
    footer: {
      style: "light",
      links: [
        {
          title: "Documentation",
          items: [
            { label: "Intro", to: "/docs/intro" },
            { label: "Installation", to: "/docs/getting-started/installation" },
            { label: "Troubleshooting", to: "/docs/operations/troubleshooting" }
          ]
        },
        {
          title: "Integrations",
          items: [
            { label: "HORUS ROS2", to: "/docs/integration/horus-ros2" },
            { label: "HORUS MR App", to: "/docs/integration/horus-mr-app" }
          ]
        },
        {
          title: "Community",
          items: [
            { label: "Repository", href: "https://github.com/RICE-unige/horus_sdk" },
            { label: "Issues", href: "https://github.com/RICE-unige/horus_sdk/issues" }
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
      defaultMode: "dark", // Switch to dark mode default for "modern" feel
      disableSwitch: false,
      respectPrefersColorScheme: true
    }
  })
};

module.exports = config;
