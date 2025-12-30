import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export default function HomepageHero() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      <div className={styles.heroInner}>
        <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        <div className={styles.heroButtons}>
          <Link
            className={`button button--primary button--lg ${styles.heroButton}`}
            to="/docs/index"
          >
            Start Learning
          </Link>
          <Link
            className={`button button--secondary button--lg ${styles.heroButtonSecondary}`}
            to="/docs/module-1-robotic-nervous-system/chapter-1-1-introduction-ros2"
          >
            Jump to Module 1
          </Link>
        </div>
        <div className={styles.heroStats}>
          <div className={styles.stat}>
            <span className={styles.statNumber}>4</span>
            <span className={styles.statLabel}>Modules</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>24+</span>
            <span className={styles.statLabel}>Chapters</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNumber}>100%</span>
            <span className={styles.statLabel}>Verified</span>
          </div>
        </div>
      </div>
    </header>
  );
}
