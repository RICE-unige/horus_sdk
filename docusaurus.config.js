// @ts-check

const lightCodeTheme = {
  plain: {
    color: "#2f2924",
    backgroundColor: "#f1eadf"
  },
  styles: [
    { types: ["comment", "prolog", "doctype", "cdata"], style: { color: "#85796e", fontStyle: "italic" } },
    { types: ["punctuation"], style: { color: "#6b6056" } },
    { types: ["property", "tag", "constant", "symbol", "deleted"], style: { color: "#9b5643" } },
    { types: ["boolean", "number"], style: { color: "#8f6931" } },
    { types: ["selector", "attr-name", "string", "char", "builtin", "inserted"], style: { color: "#647b4f" } },
    { types: ["operator", "entity", "url", "variable"], style: { color: "#6b6056" } },
    { types: ["atrule", "attr-value", "function", "class-name"], style: { color: "#7d6846" } },
    { types: ["keyword"], style: { color: "#8f6931", fontWeight: "600" } },
    { types: ["regex", "important"], style: { color: "#b36c52" } }
  ]
};

const darkCodeTheme = {
  plain: {
    color: "#ece6dc",
    backgroundColor: "#171716"
  },
  styles: [
    { types: ["comment", "prolog", "doctype", "cdata"], style: { color: "#8b8276", fontStyle: "italic" } },
    { types: ["punctuation"], style: { color: "#c6bcae" } },
    { types: ["property", "tag", "constant", "symbol", "deleted"], style: { color: "#d4866f" } },
    { types: ["boolean", "number"], style: { color: "#d6b56d" } },
    { types: ["selector", "attr-name", "string", "char", "builtin", "inserted"], style: { color: "#97b77f" } },
    { types: ["operator", "entity", "url", "variable"], style: { color: "#c6bcae" } },
    { types: ["atrule", "attr-value", "function", "class-name"], style: { color: "#c5ae79" } },
    { types: ["keyword"], style: { color: "#d6b56d", fontWeight: "600" } },
    { types: ["regex", "important"], style: { color: "#d99a6c" } }
  ]
};

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
          editUrl: "https://github.com/RICE-unige/horus_sdk/tree/main/"
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
        { to: "/docs/tutorials/summary", label: "Tutorials", position: "left" },
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
          title: "Start here",
          items: [
            { label: "Overview", to: "/docs/intro" },
            { label: "Installation", to: "/docs/getting-started/installation" },
            { label: "Quickstart", to: "/docs/getting-started/quickstart" }
          ]
        },
        {
          title: "Tutorials",
          items: [
            { label: "Tutorial summary", to: "/docs/tutorials/summary" },
            { label: "First ground robot", to: "/docs/tutorials/first-ground-robot" },
            { label: "Robot description", to: "/docs/tutorials/robot-description" }
          ]
        },
        {
          title: "Reference",
          items: [
            { label: "Curated examples", to: "/docs/examples/registration-flows" },
            { label: "Robot model", to: "/docs/python-sdk/robot-model" },
            { label: "DataViz", to: "/docs/python-sdk/dataviz" }
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
