import React, {useEffect, useRef} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import HeroScene from '@site/src/components/Landing/HeroScene';
import {ModuleArt} from '@site/src/components/Landing/ModuleArt';
import {openAssistant} from '@site/src/components/ChatWidget/events';
import {loadUrduFont} from '@site/src/utils/urduFont';
import {useProgress} from '@site/src/utils/progress';
import 'lenis/dist/lenis.css';
import styles from './index.module.css';

// The course path, grouped into four stages (each card links its modules). Module folders and slugs come from
// docs/plans/2026-10-27-proprio-course.md; URLs drop the number prefixes.
const STAGES = [
  {
    n: '00–02',
    art: 1,
    title: 'Build the foundations',
    focus: 'The math, Python and Linux you need, then how real robots are put together: frames, kinematics, dynamics and control.',
    modules: [
      ['Start here', 'start-here'],
      ['Foundations', 'foundations'],
      ['Robot fundamentals', 'robot-fundamentals'],
    ],
  },
  {
    n: '03–04',
    art: 0,
    title: 'Wire the nervous system',
    focus: 'ROS 2 from first node to full robot description, then simulated worlds in Gazebo and Isaac Sim to practise safely.',
    modules: [
      ['ROS 2', 'ros2'],
      ['Simulation', 'simulation'],
    ],
  },
  {
    n: '05–06',
    art: 2,
    title: 'See and move',
    focus: 'Cameras, LiDAR, state estimation and SLAM, then planning, Nav2 navigation, MoveIt 2 manipulation and grasping.',
    modules: [
      ['Perception', 'perception'],
      ['Motion', 'motion'],
    ],
  },
  {
    n: '07–10',
    art: 3,
    title: 'Learn, walk and reason',
    focus: 'Imitation and reinforcement learning, humanoid balance and locomotion, vision-language-action models, and the capstone.',
    modules: [
      ['Robot learning', 'robot-learning'],
      ['Humanoids', 'humanoids'],
      ['Foundation models', 'foundation-models'],
      ['Capstone & career', 'capstone-and-career'],
    ],
  },
];

const WEEKS = [
  ['Weeks 1–2', 'Start here + foundations', 'Set up your workstation; Python, linear algebra, probability and the Linux toolbelt.', 'start-here'],
  ['Weeks 3–4', 'Robot fundamentals', 'Frames, kinematics, dynamics and PID control on a simulated arm.', 'robot-fundamentals'],
  ['Weeks 5–6', 'ROS 2', 'Nodes, topics, actions, TF2 and URDF: a whole robot that talks to itself.', 'ros2'],
  ['Week 7', 'Simulation', 'Gazebo worlds, simulated sensors, Isaac Sim, and the sim-to-real gap.', 'simulation'],
  ['Weeks 8–9', 'Perception', 'Images, point clouds, detection, Kalman filters and SLAM.', 'perception'],
  ['Week 10', 'Motion', 'Path planning, Nav2, MoveIt 2 and grasping.', 'motion'],
  ['Weeks 11–12', 'Robot learning', 'Imitation learning, reinforcement learning and training in simulation.', 'robot-learning'],
  ['Week 13', 'Humanoids', 'Balance, walking, whole-body control and dexterous hands.', 'humanoids'],
  ['Week 14', 'Foundation models', 'LLM planners, vision-language grounding and VLA models.', 'foundation-models'],
  ['Weeks 15–16', 'Capstone & career', 'Ship a voice-commanded robot assistant and turn it into a portfolio.', 'capstone-and-career'],
];

const STATS = [
  ['11', 'modules'],
  ['52', 'lessons'],
  ['16', 'weeks'],
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
  const {last} = useProgress(); // "continue where you left off" once someone has started
  return (
    <section className={clsx(styles.darkStage, styles.hero)}>
      <div className={styles.heroGrid}>
        <div className={styles.heroCopy}>
          <p className={styles.kicker}>
            <span className={styles.liveDot} aria-hidden="true" />
            Proprio · the open course in Physical AI
          </p>
          <h1 className={styles.heroTitle}>
            Give AI a <em>body</em>.
          </h1>
          <p className={styles.heroLede}>
            Go from zero to expert in Physical AI and humanoid robotics: robot fundamentals, ROS 2, simulation,
            perception, robot learning and vision-language-action models. Rivet, your AI tutor, has read every lesson.
          </p>
          <div className={styles.ctas}>
            {last ? (
              <Link className={clsx('button button--primary button--lg', styles.cta)} to={last.path} title={last.title}>
                Continue: {last.title.length > 34 ? `${last.title.slice(0, 32)}…` : last.title}
              </Link>
            ) : (
              <Link className={clsx('button button--primary button--lg', styles.cta)} to="/docs/">
                Start reading
              </Link>
            )}
            <button type="button" className={clsx('button button--secondary button--lg', styles.cta)} onClick={() => openAssistant()}>
              Ask Rivet
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
            Zero to expert, <em>layer by layer</em>
          </h2>
          <p className={styles.sectionLede}>
            Eleven modules in the order a robot needs them: a body you understand, a nervous system, senses and
            movement, and finally the judgement to learn and act on what it is told.
          </p>
        </header>
        <div className={styles.bento}>
          {STAGES.map((stage, i) => (
            <article key={stage.title} className={clsx(styles.card, styles[`card${i + 1}`], 'pa-reveal')} onPointerMove={onSpotlight}>
              <div className={styles.cardArt}>
                <ModuleArt index={stage.art} />
              </div>
              <div className={styles.cardBody}>
                <p className={styles.cardMeta}>
                  <span className={styles.mono}>Modules {stage.n}</span>
                </p>
                <h3 className={styles.cardTitle}>
                  <Link to={`/docs/${stage.modules[0][1]}`} className={styles.cardLink}>
                    {stage.title}
                  </Link>
                </h3>
                <p className={styles.cardText}>{stage.focus}</p>
                <ol className={styles.lessons}>
                  {stage.modules.map(([title, slug]) => (
                    <li key={slug}>
                      <Link to={`/docs/${slug}`}>{title}</Link>
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
            Sixteen weeks, <em>from sensor to speech</em>
          </h2>
          <p className={styles.sectionLede}>
            Follow it as a semester or dip in by topic. Each block opens with clear objectives and ends with something
            that runs.
          </p>
          <Link className="button button--secondary" to="/docs/start-here">
            Open week 1
          </Link>
        </header>
        <ol className={styles.timeline}>
          {WEEKS.map(([when, title, text, slug]) => (
            <li key={slug} className={clsx(styles.week, 'pa-reveal')}>
              <span className={styles.weekWhen}>{when}</span>
              <h3 className={styles.weekTitle}>
                <Link to={`/docs/${slug}`}>{title}</Link>
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
              <h3 className={styles.cardTitle}>Rivet, a tutor that cites the course</h3>
              <p className={styles.cardText}>
                Rivet answers from the lessons themselves, knows which page you are on, and links back to the
                sections it used. Select any text in a lesson to ask about it.
              </p>
              <button type="button" className="button button--primary" onClick={() => openAssistant('When should I use a ROS 2 service instead of a topic?')}>
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
          Your first lesson takes <em>an evening</em>.
        </h2>
        <p className={styles.sectionLede}>Start with how the course works and set up your workstation. No robot needed.</p>
        <div className={styles.ctas}>
          <Link className={clsx('button button--primary button--lg', styles.cta)} to="/docs/start-here/how-this-course-works">
            Start lesson 1
          </Link>
          <Link className={clsx('button button--secondary button--lg', styles.cta)} to="/docs/">
            See all chapters
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  useSmoothScroll();
  return (
    <Layout
      title="Proprio · The open course in Physical AI & humanoid robotics"
      description="Go from zero to expert in Physical AI and humanoid robotics: robot fundamentals, ROS 2, simulation, perception, robot learning, humanoids and vision-language-action models, with Rivet, an AI tutor.">
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
