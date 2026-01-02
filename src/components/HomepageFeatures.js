import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

// Inline SVG components
const Ros2Logo = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" className={styles.featureSvg}>
    <rect width="100" height="100" fill="#1a73e8" />
    <text x="50" y="55" font-family="Arial" font-size="20" fill="white" text-anchor="middle">ROS2</text>
  </svg>
);

const SimulationLogo = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" className={styles.featureSvg}>
    <rect width="100" height="100" fill="#3cba54" />
    <text x="50" y="55" font-family="Arial" font-size="16" fill="white" text-anchor="middle">SIM</text>
  </svg>
);

const AiLogo = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" className={styles.featureSvg}>
    <rect width="100" height="100" fill="#ff6d01" />
    <text x="50" y="55" font-family="Arial" font-size="16" fill="white" text-anchor="middle">AI</text>
  </svg>
);

const FeatureList = [
  {
    title: 'ROS 2 Nervous System',
    Svg: Ros2Logo,
    description: (
      <>
        Learn how ROS 2 serves as the communication backbone for robotic systems,
        enabling distributed computing and real-time coordination between components.
      </>
    ),
  },
  {
    title: 'Simulation Environments',
    Svg: SimulationLogo,
    description: (
      <>
        Master Gazebo and NVIDIA Isaac Sim for safe development and testing of
        robotic systems before deployment to real hardware.
      </>
    ),
  },
  {
    title: 'AI-Powered Perception',
    Svg: AiLogo,
    description: (
      <>
        Discover how NVIDIA Isaac transforms robots into intelligent agents with
        advanced perception and decision-making capabilities.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}