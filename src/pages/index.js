import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import HomepageHero from '@site/src/components/HomepageHero';
import ModuleCard from '@site/src/components/ModuleCard';
import styles from './index.module.css';

// Module data for the book
const modules = [
  {
    id: 'module-1',
    title: 'Module 1 — The Robotic Nervous System',
    subtitle: 'ROS 2',
    description: 'Learn ROS 2 architecture, nodes, topics, services, and sensor integration for humanoid robots.',
    icon: '🤖',
    color: '#3B82F6',
    link: '/docs/module-1-robotic-nervous-system/chapter-1-1-introduction-ros2',
    chapters: 6,
  },
  {
    id: 'module-2',
    title: 'Module 2 — The Digital Twin',
    subtitle: 'Gazebo & Unity',
    description: 'Master physics simulation, robot worlds, sensor simulation, and sim-to-real transfer concepts.',
    icon: '🌐',
    color: '#10B981',
    link: '/docs/module-2-digital-twin/chapter-2-1-physics-simulation',
    chapters: 6,
  },
  {
    id: 'module-3',
    title: 'Module 3 — The AI-Robot Brain',
    subtitle: 'NVIDIA Isaac',
    description: 'Explore NVIDIA Isaac platform, VSLAM, perception, navigation, and synthetic data generation.',
    icon: '🧠',
    color: '#8B5CF6',
    link: '/docs/module-3-ai-robot-brain/chapter-3-1-nvidia-isaac-platform',
    chapters: 6,
  },
  {
    id: 'module-4',
    title: 'Module 4 — Vision-Language-Action',
    subtitle: 'VLA',
    description: 'Build voice command systems, LLM cognitive planning, and complete voice-to-action pipelines.',
    icon: '🎯',
    color: '#F59E0B',
    link: '/docs/module-4-vision-language-action/chapter-4-1-vla-systems',
    chapters: 6,
  },
];

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Learning Modules</h2>
          <p className={styles.sectionSubtitle}>
            Master Physical AI and Humanoid Robotics through our comprehensive, hands-on curriculum
          </p>
        </div>
        <div className={styles.moduleGrid}>
          {modules.map((module) => (
            <ModuleCard key={module.id} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CapstoneSection() {
  return (
    <section className={styles.capstone}>
      <div className={styles.container}>
        <div className={styles.capstoneContent}>
          <div className={styles.capstoneText}>
            <span className={styles.capstoneLabel}>Capstone Project</span>
            <h2 className={styles.capstoneTitle}>Build a Complete Autonomous Humanoid</h2>
            <p className={styles.capstoneDescription}>
              Apply everything you've learned to build an integrated system capable of receiving voice commands,
              planning actions, navigating environments, and manipulating objects.
            </p>
            <a
              href="/docs/capstone-project/autonomous-humanoid-implementation"
              className={`button button--primary button--lg ${styles.capstoneButton}`}
            >
              View Capstone Project
            </a>
          </div>
          <div className={styles.capstoneVisual}>
            <div className={styles.capstoneIcon}>🚀</div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={null}
      description={siteConfig.tagline}
      noFooter
      wrapperClassName="homepage-wrapper"
    >
      <HomepageHero />
      <main>
        <ModulesSection />
        <CapstoneSection />
      </main>
    </Layout>
  );
}
