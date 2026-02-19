import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

export default function Home() {
  return (
    <Layout
      title="Home"
      description="Holistic Operational Reality for Unified Systems"
    >
      <header className="hero">
        <div className="container">
          <h1 className="hero__title">
            Horus SDK
          </h1>
          <p className="hero__subtitle">
            An interface for managing multi robot system with mixed reality through a minimap base station.
          </p>
          <div className="margin-top--lg" style={{ display: "flex", gap: "1rem" }}>
            <Link
              className="button button--primary button--lg"
              to="/docs/getting-started/quickstart"
              style={{ padding: "0.8rem 2rem" }}
            >
              Start Building
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro"
              style={{ padding: "0.8rem 2rem", background: "rgba(255,255,255,0.1)" }}
            >
              Read the Docs
            </Link>
          </div>
        </div>
      </header>

      <main className="container padding-vert--xl">
        <section className="feature-grid">
          <Link to="/docs/getting-started/quickstart" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>🚀 Quickstart</h3>
            <p>Get up and running with the Horus SDK in minutes. Installation guides and "Hello World" examples.</p>
          </Link>

          <Link to="/docs/python-sdk/robot-model" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>🤖 Robot Model</h3>
            <p>Define and manipulate robot states, standardizing how your fleet interacts with the world.</p>
          </Link>

          <Link to="/docs/integration/horus-ros2" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>⚡ ROS2 Integration</h3>
            <p>Seamlessly bridge ROS2 topics with Horus runtime for real-time visualization and control.</p>
          </Link>

          <Link to="/docs/integration/horus-mr-app" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>🥽 Mixed Reality</h3>
            <p> Visualize spatial data, occupancy grids, and tf trees directly in your physical environment.</p>
          </Link>

          <Link to="/docs/examples/topic-monitoring-dashboard" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>📊 Observability</h3>
            <p>Monitor system health, topic rates, and latency with built-in dashboarding tools.</p>
          </Link>

          <Link to="/docs/reference/implementation-status" className="feature-card" style={{ textDecoration: 'none', color: 'inherit' }}>
            <h3>🛠️ Architecture</h3>
            <p>Deep dive into the system boundary, runtime flow, and parity status across languages.</p>
          </Link>
        </section>
      </main>
    </Layout>
  );
}
