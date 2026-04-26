/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docsSidebar: [
    "intro",
    {
      type: "category",
      label: "Start Here",
      items: [
        "getting-started/installation",
        "getting-started/quickstart",
        "getting-started/installer",
        "architecture/system-boundary",
        "architecture/runtime-flow"
      ]
    },
    {
      type: "category",
      label: "Tutorial Track",
      items: [
        "tutorials/summary",
        "tutorials/first-ground-robot",
        "tutorials/cameras-and-views",
        "tutorials/operator-controls",
        "tutorials/dataviz-layers",
        "tutorials/robot-description",
        "tutorials/global-maps",
        "tutorials/live-robot-checklist"
      ]
    },
    {
      type: "category",
      label: "Python SDK Reference",
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
      label: "Curated Examples",
      items: [
        "examples/registration-flows",
        "examples/camera-transport-profiles",
        "examples/occupancy-grid-workflow",
        "examples/topic-monitoring-dashboard"
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
