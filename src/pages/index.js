import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

export default function Home() {
  return (
    <Layout
      title="HORUS SDK Docs"
      description="Docusaurus docs for HORUS SDK, registration orchestration, and MR integration"
    >
      <header className="hero">
        <div className="container padding-vert--xl">
          <h1 className="hero__title">HORUS SDK Documentation</h1>
          <p className="hero__subtitle">
            Research-grade SDK docs for mixed-reality multi-robot management workflows:
            registration, observability, transport profiles, and integration with HORUS ROS2 and MR.
          </p>
          <div className="margin-top--lg">
            <Link className="button button--primary button--lg margin-right--sm" to="/docs/intro">
              Open Documentation
            </Link>
            <Link className="button button--secondary button--lg" to="/docs/getting-started/quickstart">
              Quickstart
            </Link>
          </div>
          <section className="feature-grid">
            <article className="feature-card">
              <h3>Main-Aligned Content</h3>
              <p>Documentation is rebuilt against the current SDK codebase and runtime behavior.</p>
            </article>
            <article className="feature-card">
              <h3>Integration First</h3>
              <p>Covers end-to-end flows from fake data and registration to ROS2 and MR runtime checks.</p>
            </article>
            <article className="feature-card">
              <h3>Operational Clarity</h3>
              <p>Includes dashboard semantics, troubleshooting playbooks, and install/runtime guardrails.</p>
            </article>
            <article className="feature-card">
              <h3>Parity Aware</h3>
              <p>Tracks Python baseline plus C++ and Rust parity workstreams and validation scope.</p>
            </article>
          </section>
        </div>
      </header>
    </Layout>
  );
}
