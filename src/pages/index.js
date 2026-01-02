import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Bridging digital brains and physical bodies">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <section className={styles.highlights}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--6">
                <Heading as="h2">Embodied Intelligence</Heading>
                <p>
                  This course bridges the gap between digital AI systems and physical robotic platforms, focusing on embodied intelligence principles.
                  Learn how intelligence emerges through the interaction between an agent and its environment.
                </p>
                <ul>
                  <li>Real-world robotics applications</li>
                  <li>Safety-first development practices</li>
                  <li>Simulation-to-reality transfer</li>
                  <li>Multimodal AI integration</li>
                </ul>
              </div>
              <div className="col col--6">
                <Heading as="h2">Course Structure</Heading>
                <p>
                  The curriculum is organized into four progressive modules that build upon each other to create a comprehensive understanding of embodied intelligence.
                </p>
                <div className="card">
                  <div className="card__body">
                    <h4>Module 1: ROS 2 Nervous System</h4>
                    <p>Core communication and architecture patterns</p>

                    <h4>Module 2: Simulation Environments</h4>
                    <p>Gazebo and Unity platforms for safe development</p>

                    <h4>Module 3: NVIDIA Isaac Brain</h4>
                    <p>Advanced perception and decision-making capabilities</p>

                    <h4>Module 4: Vision-Language-Action Systems</h4>
                    <p>Multimodal AI integration for human-robot interaction</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.capstone}>
          <div className="container padding-vert--lg text--center">
            <Heading as="h2">Capstone Project</Heading>
            <p className="padding-horiz--md">
              Integrate all learned concepts to create an intelligent robotic assistant that demonstrates vision-language-action capabilities
              in both simulation and real-world environments.
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/capstone-project/project-overview">
              Explore Capstone Project
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}