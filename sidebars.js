/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docsSidebar: [
    "intro",
    {
      type: "category",
      label: "Getting Started",
      items: [
        "getting-started/installation",
        "getting-started/quickstart",
        "getting-started/installer"
      ]
    },
    {
      type: "category",
      label: "Architecture",
      items: ["architecture/system-boundary", "architecture/runtime-flow"]
    },
    {
      type: "category",
      label: "Python SDK",
      items: [
        "python-sdk/robot-model",
        "python-sdk/sensors",
        "python-sdk/dataviz",
        "python-sdk/registration",
        "python-sdk/topic-monitoring"
      ]
    },
    {
      type: "category",
      label: "Examples",
      items: [
        "examples/registration-flows",
        "examples/camera-transport-profiles",
        "examples/topic-monitoring-dashboard",
        "examples/occupancy-grid-workflow"
      ]
    },
    {
      type: "category",
      label: "Integration",
      items: ["integration/horus-ros2", "integration/horus-mr-app"]
    },
    {
      type: "category",
      label: "Reference",
      items: ["reference/implementation-status", "reference/known-limitations"]
    },
    {
      type: "category",
      label: "Operations",
      items: ["operations/troubleshooting"]
    },
    {
      type: "category",
      label: "Contributing",
      items: ["contributing/docs-maintenance"]
    }
  ]
};

module.exports = sidebars;
