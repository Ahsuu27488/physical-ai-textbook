import MDXComponents from '@theme-original/MDXComponents';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Details from '@theme/Details';
import {Exercise, KeyIdea, LessonMeta, Objectives, Quiz, Solution, Steps, Takeaways, Term} from '@site/src/components/Lesson';

// Components every lesson can use without an import (keep in sync with frontend/scripts/check-mdx.mjs).
export default {
  ...MDXComponents,
  Tabs,
  TabItem,
  Details,
  LessonMeta,
  Objectives,
  KeyIdea,
  Term,
  Steps,
  Exercise,
  Solution,
  Takeaways,
  Quiz,
};
