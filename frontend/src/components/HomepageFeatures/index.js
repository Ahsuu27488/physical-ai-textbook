import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Embodied Intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn how AI systems function in the physical world, bridging the gap between digital brains and physical bodies.
        Understand the principles of embodied intelligence and how physical interaction shapes cognition.
      </>
    ),
  },
  {
    title: 'ROS 2 - The Robotic Nervous System',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master the Robot Operating System (ROS 2), the middleware framework that enables communication between
        different components of a robotic system. Build complex robotic applications with distributed computing.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac & Advanced Perception',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Explore NVIDIA Isaac platform for developing AI-powered robots with advanced perception,
        simulation, and deployment capabilities for humanoid robotics.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
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
