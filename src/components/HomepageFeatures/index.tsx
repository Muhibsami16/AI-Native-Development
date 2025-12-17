import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  imageUrl: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    imageUrl: 'https://xpert.digital/wp-content/uploads/2025/06/virtuelles-gehirn-robotik-Xpert.Digital-png.png', // Yahan apni image ka URL paste karen
    description: (
      <>
        Learn the fundamentals of ROS 2 for Physical AI & Humanoid Robotics. Master the robotic nervous system with comprehensive tutorials on ROS 2 concepts, communication patterns, and node management.
      </>
    ),
    link: '/docs/module-1',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    imageUrl: 'https://ichef.bbci.co.uk/news/1024/cpsprodpb/8145/production/_125339033_gettyimages-109685978.jpg.webp', // Yahan apni image ka URL paste karen
    description: (
      <>
        Explore digital twin technologies for robotics simulation. Learn to create realistic virtual environments using Gazebo and Unity for testing and training humanoid robots.
      </>
    ),
    link: '/docs/module-2',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    imageUrl: 'https://img.pikbest.com/illustration/20250721/ai-robot-with-glowing-brain-humanoid-interacting-artificial-intelligence_11799465.jpg!w700wp', // Yahan apni image ka URL paste karen
    description: (
      <>
        Dive into NVIDIA Isaac ecosystem for advanced perception and navigation. Master Isaac Sim, Isaac ROS perception, and Nav2 for humanoid navigation in this cutting-edge AI robotics module.
      </>
    ),
    link: '/docs/module-3-isaac-ai-robot',
  },
];

function Feature({ title, imageUrl, description, link }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <Link to={link} className={styles.featureLink}>
        <div className="text--center">
          <img
            src={imageUrl}
            alt={title}
            className={styles.featureSvg}
            role="img"
          />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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