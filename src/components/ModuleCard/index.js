import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function ModuleCard({ module }) {
  const { id, title, subtitle, description, icon, color, link, chapters } = module;

  return (
    <Link to={link} className={styles.cardLink}>
      <article
        className={styles.card}
        style={{ '--module-color': color }}
      >
        <div className={styles.cardHeader}>
          <span className={styles.icon}>{icon}</span>
          <span className={styles.badge}>{chapters} chapters</span>
        </div>
        <div className={styles.cardBody}>
          <h3 className={styles.title}>{title}</h3>
          <p className={styles.subtitle}>{subtitle}</p>
          <p className={styles.description}>{description}</p>
        </div>
        <div className={styles.cardFooter}>
          <span className={styles.cta}>
            Start Learning
            <svg
              className={styles.arrow}
              width="16"
              height="16"
              viewBox="0 0 16 16"
              fill="currentColor"
            >
              <path d="M6.22 3.22a.75.75 0 011.06 0l4.25 4.25a.75.75 0 010 1.06l-4.25 4.25a.75.75 0 01-1.06-1.06L9.94 8 6.22 4.28a.75.75 0 010-1.06z" />
            </svg>
          </span>
        </div>
      </article>
    </Link>
  );
}
