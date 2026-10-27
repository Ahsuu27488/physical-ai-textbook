import React, {useEffect, useRef} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import HeroScene from '@site/src/components/Landing/HeroScene';
import {ModuleArt} from '@site/src/components/Landing/ModuleArt';
import {openAssistant} from '@site/src/components/ChatWidget/events';
import {loadUrduFont} from '@site/src/utils/urduFont';
import 'lenis/dist/lenis.css';
import styles from './index.module.css';

const MODULES = [
  {
    id: 'module-1',
    n: '01',
    title: 'The robotic nervous system',
    tool: 'ROS 2',
    focus: 'Nodes, topics, services and actions: the middleware every part of a robot talks through.',
    lessons: [
      ['ROS 2 architecture and DDS', 'lesson-1-ros2-architecture'],
      ['Nodes, topics and services', 'lesson-2-nodes-topics-services'],
      ['Actions and launch files', 'lesson-3-actions-and-launch-files'],
      ['URDF and robot description', 'lesson-4-urdf-and-robot-description'],
    ],
  },
  {
    id: 'module-2',
    n: '02',
    title: 'The digital twin',
    tool: 'Gazebo & Unity',
    focus: 'Physics simulation and virtual sensors, so you can break robots for free.',
    lessons: [
      ['Gazebo basics', 'lesson-1-gazebo-basics'],
      ['Sensor simulation', 'lesson-2-sensor-simulation'],
      ['ROS–Gazebo integration', 'lesson-3-ros-gazebo-integration'],
    ],
  },
  {
    id: 'module-3',
    n: '03',
    title: 'The AI-robot brain',
    tool: 'NVIDIA Isaac',
    focus: 'Perception, synthetic data and GPU-accelerated ROS for robots that see.',
    lessons: [
      ['Isaac Sim basics', 'lesson-1-isaac-sim-basics'],
      ['Synthetic data generation', 'lesson-2-synthetic-data-generation'],
      ['Isaac ROS integration', 'lesson-3-isaac-ros-integration'],
    ],
  },
  {
    id: 'module-4',
    n: '04',
    title: 'Vision, language, action',
    tool: 'VLA models',
    focus: 'Where large language models meet motors: plans from words, actions from pixels.',
    lessons: [
      ['Vision-language fundamentals', 'lesson-1-vision-language-fundamentals'],
      ['Cognitive planning', 'lesson-2-cognitive-planning'],
      ['VLA integration', 'lesson-3-vla-integration'],
    ],
  },
];

const WEEKS = [
  ['Weeks 1–2', 'Introduction to Physical AI', 'What embodied intelligence is, and why a body changes the problem.', 'week-01-02'],
  ['Weeks 3–5', 'ROS 2 fundamentals', 'Build nodes, wire topics and services, launch a whole robot.', 'week-03-05'],
  ['Weeks 6–7', 'Robot simulation with Gazebo', 'Model a robot and its sensors, then drive it in a simulated world.', 'week-06-07'],
  ['Weeks 8–10', 'NVIDIA Isaac platform', 'Perception, manipulation and the sim-to-real gap.', 'week-08-10'],
  ['Weeks 11–12', 'Humanoid robot development', 'Kinematics, dynamics, balance and walking.', 'week-11-12'],
  ['Week 13', 'Conversational robotics', 'Put it together: a robot that takes a spoken request and acts on it.', 'week-13'],
];

const STATS = [
  ['4', 'modules'],
  ['13', 'lessons'],
  ['13', 'weeks'],
];

function useSmoothScroll() {
  useEffect(() => {
    // Lenis on this page only (the docs keep native scrolling); never with reduced motion.
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return undefined;
    let lenis;
    let cancelled = false;
    import('lenis').then(({default: Lenis}) => {
      if (!cancelled) lenis = new Lenis({lerp: 0.1, smoothWheel: true, autoRaf: true});
    });
    return () => {
      cancelled = true;
      lenis?.destroy();
    };
  }, []);
}

function onSpotlight(e) {
  const r = e.currentTarget.getBoundingClientRect();
  e.currentTarget.style.setProperty('--mx', `${e.clientX - r.left}px`);
  e.currentTarget.style.setProperty('--my', `${e.clientY - r.top}px`);
}

function Hero() {
  return (
    <section className={clsx(styles.darkStage, styles.hero)}>
      <div className={styles.heroGrid}>
        <div className={styles.heroCopy}>
          <p className={styles.kicker}>
            <span className={styles.liveDot} aria-hidden="true" />
            Open textbook · ROS 2 to vision-language-action
          </p>
          <h1 className={styles.heroTitle}>
            Give AI a <em>body</em>.
          </h1>
          <p className={styles.heroLede}>
            A hands-on course in Physical AI and humanoid robotics: ROS 2, simulation in Gazebo and NVIDIA Isaac,
            and the models that turn language into motion. An assistant that has read every chapter answers as you go.
          </p>
          <div className={styles.ctas}>
            <Link className={clsx('button button--primary button--lg', styles.cta)} to="/docs/">
              Start reading
            </Link>
            <button type="button" className={clsx('button button--secondary button--lg', styles.cta)} onClick={() => openAssistant()}>
              Ask the assistant
            </button>
          </div>
          <dl className={styles.stats}>
            {STATS.map(([value, label]) => (
              <div key={label}>
                <dt>{label}</dt>
                <dd>{value}</dd>
              </div>
            ))}
          </dl>
        </div>
        <figure className={styles.heroStage}>
          <HeroScene />
          <span className={clsx(styles.bracket, styles.tl)} />
          <span className={clsx(styles.bracket, styles.tr)} />
          <span className={clsx(styles.bracket, styles.bl)} />
          <span className={clsx(styles.bracket, styles.br)} />
          <figcaption className={styles.caption}>fig. 1 — a humanoid as a lidar sees it, rendered live</figcaption>
        </figure>
      </div>
    </section>
  );
}

function Modules() {
  return (
    <section className={styles.section} aria-labelledby="modules-title">
      <div className={styles.container}>
        <header className={clsx(styles.sectionHead, 'pa-reveal')}>
          <h2 id="modules-title" className={styles.h2}>
            The stack, <em>layer by layer</em>
          </h2>
          <p className={styles.sectionLede}>
            Four modules, in the order a robot needs them: a way to talk, a world to practise in, eyes, and finally
            the judgement to act on what it is told.
          </p>
        </header>
        <div className={styles.bento}>
          {MODULES.map((m, i) => (
            <article key={m.id} className={clsx(styles.card, styles[`card${i + 1}`], 'pa-reveal')} onPointerMove={onSpotlight}>
              <div className={styles.cardArt}>
                <ModuleArt index={i} />
              </div>
              <div className={styles.cardBody}>
                <p className={styles.cardMeta}>
                  <span className={styles.mono}>{m.n}</span>
                  <span>{m.tool}</span>
                </p>
                <h3 className={styles.cardTitle}>
                  <Link to={`/docs/${m.id}/intro`} className={styles.cardLink}>
                    {m.title}
                  </Link>
                </h3>
                <p className={styles.cardText}>{m.focus}</p>
                <ol className={styles.lessons}>
                  {m.lessons.map(([title, slug]) => (
                    <li key={slug}>
                      <Link to={`/docs/${m.id}/${slug}`}>{title}</Link>
                    </li>
                  ))}
                </ol>
              </div>
            </article>
          ))}
        </div>
      </div>
    </section>
  );
}

function Schedule() {
  return (
    <section className={clsx(styles.section, styles.scheduleSection)} aria-labelledby="schedule-title">
      <div className={clsx(styles.container, styles.schedule)}>
        <header className={styles.scheduleHead}>
          <h2 id="schedule-title" className={styles.h2}>
            Thirteen weeks, <em>from sensor to speech</em>
          </h2>
          <p className={styles.sectionLede}>
            Follow it as a semester or dip in by topic. Each block opens with clear objectives and ends with something
            that runs.
          </p>
          <Link className="button button--secondary" to="/docs/week-01-02/intro">
            Open week 1
          </Link>
        </header>
        <ol className={styles.timeline}>
          {WEEKS.map(([when, title, text, slug]) => (
            <li key={slug} className={clsx(styles.week, 'pa-reveal')}>
              <span className={styles.weekWhen}>{when}</span>
              <h3 className={styles.weekTitle}>
                <Link to={`/docs/${slug}/intro`}>{title}</Link>
              </h3>
              <p className={styles.weekText}>{text}</p>
            </li>
          ))}
        </ol>
      </div>
    </section>
  );
}

function Tools() {
  // the Urdu font is large: fetch it only as the sample approaches the viewport
  const urduRef = useRef(null);
  useEffect(() => {
    const el = urduRef.current;
    if (!el) return undefined;
    const io = new IntersectionObserver(([e]) => {
      if (e.isIntersecting) {
        loadUrduFont();
        io.disconnect();
      }
    }, {rootMargin: '600px 0px'});
    io.observe(el);
    return () => io.disconnect();
  }, []);
  return (
    <section className={styles.section} aria-labelledby="tools-title">
      <div className={styles.container}>
        <header className={clsx(styles.sectionHead, 'pa-reveal')}>
          <h2 id="tools-title" className={styles.h2}>
            Tools that <em>read along</em>
          </h2>
          <p className={styles.sectionLede}>Built into every page, so you never leave the lesson to get unstuck.</p>
        </header>
        <div className={styles.toolsGrid}>
          <article className={clsx(styles.tool, styles.toolChat, 'pa-reveal')} onPointerMove={onSpotlight}>
            <div className={styles.toolBody}>
              <h3 className={styles.cardTitle}>An assistant that cites the book</h3>
              <p className={styles.cardText}>
                Answers come from the textbook itself (retrieval over every lesson) and link back to the pages they
                used.
              </p>
              <button type="button" className="button button--primary" onClick={() => openAssistant('What is the difference between a ROS 2 topic and a service?')}>
                Try a question
              </button>
            </div>
            <div className={styles.mockChat} aria-label="Example conversation">
              <span className={styles.exampleTag}>Example</span>
              <p className={styles.mockUser}>When should I use a service instead of a topic?</p>
              <div className={styles.mockBot}>
                <p>
                  Use a <strong>service</strong> for a request that needs one reply, like “reset the arm”. Use a{' '}
                  <strong>topic</strong> for a stream, like camera frames or joint states.
                </p>
                <span className={styles.sourceChip}>Module 1 · Nodes, topics and services</span>
              </div>
            </div>
          </article>
          <article className={clsx(styles.tool, 'pa-reveal')} onPointerMove={onSpotlight}>
            <div className={styles.toolBody}>
              <h3 className={styles.cardTitle}>Read it in Urdu</h3>
              <p className={styles.cardText}>Translate any page from the Tools menu. Technical terms stay intact.</p>
            </div>
            <p ref={urduRef} className={styles.urduSample} lang="ur" dir="rtl">
              ROS 2 روبوٹ کا اعصابی نظام ہے۔
            </p>
            <p className={styles.urduGloss}>“ROS 2 is the nervous system of a robot.”</p>
          </article>
          <article className={clsx(styles.tool, 'pa-reveal')} onPointerMove={onSpotlight}>
            <div className={styles.toolBody}>
              <h3 className={styles.cardTitle}>Pitched at your level</h3>
              <p className={styles.cardText}>
                Tell it your software and hardware background once; Personalize rewrites a page for you.
              </p>
            </div>
            <div className={styles.levels} aria-hidden="true">
              <span>Beginner</span>
              <span data-on="true">Intermediate</span>
              <span>Advanced</span>
            </div>
          </article>
        </div>
      </div>
    </section>
  );
}

function Closing() {
  return (
    <section className={clsx(styles.darkStage, styles.closing)}>
      <div className={clsx(styles.container, styles.closingInner, 'pa-reveal')}>
        <h2 className={styles.closingTitle}>
          Lesson 1 takes <em>an evening</em>.
        </h2>
        <p className={styles.sectionLede}>Start with how ROS 2 moves messages around a robot. No hardware needed.</p>
        <div className={styles.ctas}>
          <Link className={clsx('button button--primary button--lg', styles.cta)} to="/docs/module-1/lesson-1-ros2-architecture">
            Start lesson 1
          </Link>
          <Link className={clsx('button button--secondary button--lg', styles.cta)} to="/docs/">
            See all chapters
          </Link>
        </div>
      </div>
      <p className={styles.wordmark} aria-hidden="true">
        Physical AI
      </p>
    </section>
  );
}

export default function Home() {
  useSmoothScroll();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A hands-on textbook for Physical AI and humanoid robotics: ROS 2, Gazebo, NVIDIA Isaac and vision-language-action models, with a built-in assistant.">
      <Head>
        {/* lets the global navbar styles know it sits on the dark hero (see custom.css) */}
        <html className="pa-landing" />
      </Head>
      <main className={styles.page}>
        <Hero />
        <Modules />
        <Schedule />
        <Tools />
        <Closing />
      </main>
    </Layout>
  );
}
