import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

const workflowCards = [
  {
    title: "Ground robot ops",
    href: "/docs/getting-started/quickstart",
    description:
      "Register a small wheeled fleet with Robot Manager, teleop, navigation tasks, paths, odometry, and collision overlays."
  },
  {
    title: "Robot description and maps",
    href: "/docs/examples/occupancy-grid-workflow",
    description:
      "Move from simple dimensions to URDF-backed bodies and global occupancy, pointcloud, mesh, and octomap layers."
  },
  {
    title: "Live systems",
    href: "/docs/examples/topic-monitoring-dashboard",
    description:
      "Use the Carter and Unitree Go1 examples as production-oriented registration references for real ROS graphs."
  }
];

const guideCards = [
  {
    title: "Robot model",
    href: "/docs/python-sdk/robot-model",
    description: "Define robot identity, base frames, control surfaces, and runtime-facing metadata."
  },
  {
    title: "Sensors",
    href: "/docs/python-sdk/sensors",
    description: "Register cameras and lidar with the transport, frame, and view settings HORUS MR expects."
  },
  {
    title: "DataViz",
    href: "/docs/python-sdk/dataviz",
    description: "Declare paths, safety overlays, maps, and semantic layers without leaking Unity-specific runtime details."
  },
  {
    title: "Registration",
    href: "/docs/python-sdk/registration",
    description: "Publish robot batches, keep them alive, and understand the ACK path between the SDK and the app."
  }
];

const boundaryCards = [
  {
    title: "horus_sdk",
    description: "Robot registration models, payload serialization, observability semantics, and developer-facing examples."
  },
  {
    title: "horus_ros2",
    description: "Custom HORUS ROS 2 runtime, topic routing, bridge lifecycle, and WebRTC-capable camera transport."
  },
  {
    title: "horus",
    description: "Mixed-reality workspace lifecycle, Robot Manager, task authoring UI, and operator-facing runtime policy."
  }
];

export default function Home() {
  return (
    <Layout
      title="Documentation"
      description="Robot registration and mixed-reality integration for ROS 2 fleets."
    >
      <main className="siteHome">
        <div className="container">
          <section className="homeIntro">
            <div className="homePanel homeLead">
              <h1>HORUS SDK documentation for robotics developers</h1>
              <p>
                Use this site to register ROS 2 robots into HORUS MR, define camera and DataViz
                behavior, and validate real or simulated workflows without reverse-engineering the
                app runtime.
              </p>
              <div className="homeActions">
                <Link className="button button--primary" to="/docs/getting-started/quickstart">
                  Open quickstart
                </Link>
                <Link className="button button--secondary" to="/docs/examples/registration-flows">
                  Browse curated examples
                </Link>
              </div>
            </div>
            <div className="homePanel">
              <h2>What you build with it</h2>
              <ul className="homeList">
                <li>Robot registration for wheeled, drone, and legged platforms</li>
                <li>Camera, lidar, teleop, and navigation-task metadata</li>
                <li>Robot-scoped and global DataViz layers for MR operators</li>
                <li>Operational validation against HORUS MR and `horus_ros2`</li>
              </ul>
            </div>
          </section>

          <section className="homeSection">
            <div className="sectionHeading">
              <h2>Start with a workflow</h2>
              <p>
                The curated scripts in <code>python/examples/</code> are the primary references.
                The legacy folder remains available for paired fake runtimes and deeper validation,
                but the root examples are the ones meant to be copied into real projects.
              </p>
            </div>
            <div className="homeGrid">
              {workflowCards.map((card) => (
                <Link key={card.title} className="homeCard" to={card.href}>
                  <h3>{card.title}</h3>
                  <p>{card.description}</p>
                </Link>
              ))}
            </div>
          </section>

          <section className="homeSection">
            <div className="sectionHeading">
              <h2>Core SDK guides</h2>
              <p>
                These pages cover the Python surface area that matters when you are integrating a
                robot or adding a new runtime capability.
              </p>
            </div>
            <div className="homeGrid">
              {guideCards.map((card) => (
                <Link key={card.title} className="homeCard" to={card.href}>
                  <h3>{card.title}</h3>
                  <p>{card.description}</p>
                </Link>
              ))}
            </div>
          </section>

          <section className="homeSection">
            <div className="sectionHeading">
              <h2>Repository boundary</h2>
              <p>
                HORUS is split intentionally. The SDK does not own workspace UX or bridge internals,
                and the docs reflect that separation directly.
              </p>
            </div>
            <div className="homeGrid homeGrid--tight">
              {boundaryCards.map((card) => (
                <div key={card.title} className="homeCard">
                  <h3>{card.title}</h3>
                  <p>{card.description}</p>
                </div>
              ))}
            </div>
          </section>
        </div>
      </main>
    </Layout>
  );
}
