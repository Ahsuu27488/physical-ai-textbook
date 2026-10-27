// Compile lesson files the way Docusaurus will (MDX 3 + remark-math), without a full site build.
// Usage (from frontend/): node scripts/check-mdx.mjs docs/03-ros2            (a folder or files)
// Exit code 1 if any file fails; each failure prints file:line:col and the parser message.
import {readFileSync, readdirSync, statSync} from 'node:fs';
import {join} from 'node:path';
import {compile} from '@mdx-js/mdx';
import remarkMath from 'remark-math';
import remarkGfm from 'remark-gfm';

const KNOWN = new Set([
  'LessonMeta', 'Objectives', 'KeyIdea', 'Quiz', 'Exercise', 'Solution', 'Takeaways', 'Term', 'Steps',
  'Tabs', 'TabItem', 'Details',
]);

function files(target) {
  const st = statSync(target);
  if (st.isFile()) return [target];
  return readdirSync(target).flatMap((f) => files(join(target, f))).filter((f) => /\.mdx?$/.test(f));
}

const targets = process.argv.slice(2);
if (!targets.length) {
  console.error('usage: node scripts/check-mdx.mjs <folder-or-file>...');
  process.exit(2);
}

let failed = 0;
for (const file of targets.flatMap(files)) {
  const raw = readFileSync(file, 'utf8');
  const fm = raw.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n/);
  const problems = [];
  if (!fm) problems.push('missing front matter (--- title: ... ---)');
  else {
    if (!/^title:/m.test(fm[1])) problems.push('front matter has no title');
    if (!/^description:/m.test(fm[1])) problems.push('front matter has no description');
  }
  // keep line numbers: blank out the front matter instead of removing it
  const body = fm ? raw.replace(fm[0], fm[0].replace(/[^\n]/g, '')) : raw;
  for (const [, name] of body.matchAll(/<([A-Z][A-Za-z]*)[\s/>]/g)) {
    if (!KNOWN.has(name)) problems.push(`unknown component <${name}> (allowed: ${[...KNOWN].join(', ')})`);
  }
  try {
    await compile(body, {remarkPlugins: [remarkGfm, remarkMath]});
  } catch (err) {
    const where = err.place?.start ?? err.place ?? err.position?.start;
    problems.push(`${where ? `${where.line}:${where.column} ` : ''}${err.reason || err.message}`);
  }
  if (problems.length) {
    failed++;
    for (const p of [...new Set(problems)]) console.log(`FAIL ${file}: ${p}`);
  } else console.log(`ok   ${file}`);
}
process.exit(failed ? 1 : 0);
