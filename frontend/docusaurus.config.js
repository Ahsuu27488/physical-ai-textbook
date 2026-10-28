// @ts-check
// Site config for Kinemata, the open course in Physical AI & humanoid robotics.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// One Light / One Dark with comments raised to 4.5:1 contrast (the stock greys are ~2.3:1).
const withComments = (theme, color) => ({
  ...theme,
  styles: theme.styles.map((s) => (s.types.includes('comment') ? {...s, style: {...s.style, color}} : s)),
});

const REPO = 'https://github.com/Ahsuu27488/physical-ai-textbook';
// Vercel serves the site at '/'; GitHub Pages at '/physical-ai-textbook/' (set in deploy.yml).
const baseUrl = process.env.BASE_URL || '/';
const DESCRIPTION =
  'Kinemata is a free, hands-on course that takes you from zero to expert in Physical AI and humanoid robotics: ' +
  'robot fundamentals, ROS 2, simulation, perception, motion, robot learning, humanoids and vision-language-action models.';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Kinemata',
  tagline: 'The open course in Physical AI & humanoid robotics',
  favicon: 'img/favicon.ico',

  future: {
    v4: true, // also turns on the Rspack bundler (@docusaurus/faster), see D-007
  },

  url: process.env.SITE_URL || 'https://physical-ai-textbook-puce-two.vercel.app',
  baseUrl,

  organizationName: 'Ahsuu27488',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',
  onBrokenAnchors: 'warn',

  // Urdu is a full edition, translated once with Gemini (backend/scripts/translate_course.py) into i18n/ur;
  // pages not translated yet fall back to English. Other languages: on demand from the Tools menu.
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {label: 'English'},
      ur: {label: 'اردو', direction: 'rtl', htmlLang: 'ur'},
    },
  },

  customFields: {
    backendUrl: process.env.BACKEND_URL || 'https://ahsuu27488-physical-ai-textbook-backend.hf.space',
    assistantName: 'Rivet',
  },

  markdown: {
    mermaid: true,
  },
  themes: [
    '@docusaurus/theme-mermaid',
    [
      // offline full-text search over the lessons (built at build time, no external service)
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        indexBlog: false,
        indexPages: false,
        docsRouteBasePath: '/docs',
        highlightSearchTermsOnTargetPage: true,
        searchResultLimits: 8,
        searchBarShortcutHint: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: `${REPO}/tree/main/frontend`,
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          showLastUpdateTime: false,
        },
        blog: false, // see D-010
        theme: {
          customCss: './src/css/custom.css',
        },
        sitemap: {
          changefreq: 'weekly',
        },
      }),
    ],
  ],

  // UI fonts are self-hosted (src/css/custom.css); the Urdu font loads on demand (src/utils/urduFont.js).
  // KaTeX CSS + fonts are vendored in static/katex (fonts load only on pages with math).
  stylesheets: [{href: `${baseUrl}katex/katex.min.css`, type: 'text/css'}],

  headTags: [
    {tagName: 'link', attributes: {rel: 'icon', type: 'image/svg+xml', href: `${baseUrl}img/favicon.svg`}},
    {tagName: 'link', attributes: {rel: 'icon', type: 'image/png', sizes: '32x32', href: `${baseUrl}img/favicon-32x32.png`}},
    {tagName: 'link', attributes: {rel: 'apple-touch-icon', sizes: '180x180', href: `${baseUrl}img/apple-touch-icon.png`}},
    {tagName: 'link', attributes: {rel: 'manifest', href: `${baseUrl}img/site.webmanifest`}},
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      colorMode: {
        defaultMode: 'dark',
        respectPrefersColorScheme: true,
      },
      metadata: [
        {name: 'description', content: DESCRIPTION},
        {name: 'keywords', content: 'physical AI, humanoid robotics, ROS 2, robotics course, Gazebo, Isaac Sim, robot learning, VLA'},
        {name: 'theme-color', content: '#0b0e13'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {property: 'og:site_name', content: 'Kinemata'},
      ],
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 3,
      },
      navbar: {
        title: 'Kinemata',
        hideOnScroll: false,
        logo: {
          alt: '', // decorative: the "Kinemata" title sits right next to it in the same link
          src: 'img/logo.svg',
          width: 30,
          height: 30,
        },
        items: [
          {type: 'docSidebar', sidebarId: 'courseSidebar', position: 'left', label: 'Course'},
          {to: '/docs/start-here/how-this-course-works', label: 'How it works', position: 'left'},
          {type: 'search', position: 'right'},
          {type: 'localeDropdown', position: 'right'},
          {type: 'custom-TranslatePersonalizeButtons', position: 'right'},
          {
            href: REPO,
            position: 'right',
            className: 'pa-github-link',
            'aria-label': 'Kinemata on GitHub',
          },
          {type: 'custom-AuthButtons', position: 'right'},
        ],
      },
      // Rendered by the swizzled theme/Footer (src/theme/Footer); these are its link columns.
      footer: {
        links: [
          {
            title: 'Course',
            items: [
              {label: 'Start here', to: '/docs/'},
              {label: 'Robot fundamentals', to: '/docs/robot-fundamentals'},
              {label: 'ROS 2', to: '/docs/ros2'},
              {label: 'Humanoids', to: '/docs/humanoids'},
              {label: 'Capstone', to: '/docs/capstone-and-career'},
            ],
          },
          {
            title: 'Learn',
            items: [
              {label: 'How the course works', to: '/docs/start-here/how-this-course-works'},
              {label: 'Set up your workstation', to: '/docs/start-here/workstation-setup'},
              {label: 'The map of Physical AI', to: '/docs/start-here/map-of-physical-ai'},
            ],
          },
          {
            title: 'Project',
            items: [
              {label: 'GitHub', href: REPO},
              {label: 'Report an issue', href: `${REPO}/issues`},
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Kinemata`,
      },
      prism: {
        theme: withComments(prismThemes.oneLight, '#5f6672'),
        darkTheme: withComments(prismThemes.oneDark, '#9ba3b4'),
        additionalLanguages: ['bash', 'cmake', 'docker'],
      },
      mermaid: {
        theme: {light: 'neutral', dark: 'dark'},
      },
    }),
};

export default config;
