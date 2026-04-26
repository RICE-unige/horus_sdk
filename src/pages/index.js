import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

const startLinks = [
  {
    title: "Install",
    href: "/docs/getting-started/installation",
    description: "Prepare ROS 2, the Python environment, and the horus_ros2 workspace."
  },
  {
    title: "Quickstart",
    href: "/docs/getting-started/quickstart",
    description: "Run one working end-to-end registration before you customize anything."
  },
  {
    title: "Tutorial track",
    href: "/docs/tutorials/summary",
    description: "Learn how to build your own registration layer by layer."
  }
];

const paths = [
  {
    title: "Learn the model",
    description:
      "Use the tutorial track to understand robot identity, cameras, operator controls, DataViz, robot description, and world layers.",
    links: [
      { title: "Tutorial summary", href: "/docs/tutorials/summary" },
      { title: "First ground robot", href: "/docs/tutorials/first-ground-robot" },
      { title: "Cameras and views", href: "/docs/tutorials/cameras-and-views" }
    ]
  },
  {
    title: "Build from references",
    description:
      "Use the curated example catalog and Python API pages once you know which contract shape you need.",
    links: [
      { title: "Curated examples", href: "/docs/examples/registration-flows" },
      { title: "Robot model API", href: "/docs/python-sdk/robot-model" },
      { title: "Registration API", href: "/docs/python-sdk/registration" }
    ]
  },
  {
    title: "Integrate with the stack",
    description:
      "Use the integration pages when you need to reason about the SDK, horus_ros2, and the HORUS MR app together.",
    links: [
      { title: "System boundary", href: "/docs/architecture/system-boundary" },
      { title: "HORUS ROS 2", href: "/docs/integration/horus-ros2" },
      { title: "HORUS MR app", href: "/docs/integration/horus-mr-app" }
    ]
  }
];

export default function Home() {
  return (
    <Layout
      title="Documentation"
      description="Robot registration and mixed-reality integration for ROS 2 fleets."
    >
      <main className="siteHome">
        <div className="container homeContainer">
          <section className="homeHero">
            <p className="homeEyebrow">HORUS SDK</p>
            <h1>Documentation for robotics developers integrating ROS 2 robots into HORUS MR</h1>
            <p className="homeLeadText">
              Define robot identity, cameras, teleop, tasks, robot descriptions, and DataViz
              contracts without reverse-engineering the mixed-reality runtime.
            </p>
            <div className="homeActions">
              <Link className="button button--primary" to="/docs/getting-started/quickstart">
                Run quickstart
              </Link>
              <Link className="button button--secondary" to="/docs/tutorials/summary">
                Open tutorials
              </Link>
            </div>
          </section>

          <section className="homeSection">
            <h2>Start here</h2>
            <div className="homeGrid homeGrid--three">
              {startLinks.map((item) => (
                <Link key={item.title} className="homeCard" to={item.href}>
                  <h3>{item.title}</h3>
                  <p>{item.description}</p>
                </Link>
              ))}
            </div>
          </section>

          <section className="homeSection">
            <h2>Use the docs by intent</h2>
            <div className="homeStack">
              {paths.map((section) => (
                <div key={section.title} className="homeStrip">
                  <div>
                    <h3>{section.title}</h3>
                    <p>{section.description}</p>
                  </div>
                  <ul className="homeLinkList homeLinkList--inline">
                    {section.links.map((link) => (
                      <li key={link.title}>
                        <Link to={link.href}>{link.title}</Link>
                      </li>
                    ))}
                  </ul>
                </div>
              ))}
            </div>
          </section>

          <section className="homeSection homeSection--narrow">
            <p className="homeBoundary">
              <strong>Repository boundary:</strong> <code>horus_sdk</code> defines contracts and
              examples. <code>horus_ros2</code> owns the runtime bridge and transport.
              <code> horus</code> owns the mixed-reality workspace and Robot Manager UX.
            </p>
          </section>
        </div>
      </main>
    </Layout>
  );
}
