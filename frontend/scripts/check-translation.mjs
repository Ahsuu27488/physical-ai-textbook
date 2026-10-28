// Quality checks for a machine-translated locale (beyond "it compiles", which check-mdx.mjs covers):
//   1. letters from a foreign script mixed into the text (e.g. Gurmukhi or Devanagari letters inside Urdu words);
//   2. math that contains the locale's letters outside \text{...} (KaTeX can't typeset them).
// Code (fenced and inline) is ignored. Exit code 1 when anything is found.
//
//   node scripts/check-translation.mjs i18n/ur/docusaurus-plugin-content-docs/current   (a folder or one file)
import {readFileSync, readdirSync, statSync} from 'node:fs';
import {join, relative} from 'node:path';
import {compile} from '@mdx-js/mdx';
import remarkMath from 'remark-math';
import remarkGfm from 'remark-gfm';

const root = process.argv[2];
if (!root) {
  console.error('usage: check-translation.mjs <dir>');
  process.exit(2);
}
const LOCALE = /[؀-ۿݐ-ݿﭐ-﷿ﹰ-﻿]/; // Arabic script (Urdu)
// scripts that should never appear in an Urdu lesson (Latin, digits and punctuation are fine)
const FOREIGN = /[ऀ-ॿঀ-৿਀-੿઀-૿଀-ൿ฀-๿぀-ヿ一-鿿가-힯Ѐ-ӿ]/u;
const TEXT_CMD = /\\(?:text|mathrm|textrm|operatorname|mbox)\{[^}]*\}/g;

const files = (dir) =>
  readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    return statSync(path).isDirectory() ? files(path) : /\.mdx?$/.test(name) ? [path] : [];
  });
const targets = statSync(root).isDirectory() ? files(root) : [root];

// collect the parsed tree (after remark-math) without rendering it
const collect = (found) => () => (tree) => {
  const walk = (node) => {
    if (node.type === 'code' || node.type === 'inlineCode') return;
    if ((node.type === 'math' || node.type === 'inlineMath') && LOCALE.test(node.value.replace(TEXT_CMD, ''))) {
      found.push(['math', node.position?.start.line, node.value.slice(0, 90)]);
      return;
    }
    if (typeof node.value === 'string' && FOREIGN.test(node.value)) {
      const word = node.value.split(/\s+/).find((w) => FOREIGN.test(w));
      found.push(['script', node.position?.start.line, word]);
    }
    for (const child of node.children || []) walk(child);
  };
  walk(tree);
};

let problems = 0;
for (const file of targets) {
  const found = [];
  try {
    await compile(readFileSync(file, 'utf8'), {remarkPlugins: [remarkGfm, remarkMath, collect(found)]});
  } catch (err) {
    found.push(['compile', err.line, String(err.reason || err.message).slice(0, 90)]);
  }
  for (const [kind, line, detail] of found) console.log(`${kind.padEnd(7)} ${relative(root, file) || file}:${line ?? '?'}  ${detail}`);
  problems += found.length;
}
console.log(problems ? `${problems} problem(s)` : 'clean');
process.exit(problems ? 1 : 0);
