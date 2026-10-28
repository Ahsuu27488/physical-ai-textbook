// Turns a production build into a redirect site: for every page in `build/`, a tiny HTML file at the same path
// that sends the visitor (and search engines, via rel=canonical + an instant meta refresh) to that page on the
// main site. GitHub Pages publishes this instead of a second copy of the course (D-028).
//
//   node scripts/make-redirects.mjs <build dir> <out dir> <target origin> <base path on this host>
//   node scripts/make-redirects.mjs build pages-redirect https://physical-ai-textbook-puce-two.vercel.app /physical-ai-textbook/
import {mkdirSync, readdirSync, rmSync, statSync, writeFileSync} from 'node:fs';
import {join, relative, sep} from 'node:path';

const [buildDir, outDir, target, base = '/'] = process.argv.slice(2);
if (!buildDir || !outDir || !target) {
  console.error('usage: make-redirects.mjs <build dir> <out dir> <target origin> [base path]');
  process.exit(2);
}
const origin = target.replace(/\/+$/, '');
const prefix = `/${base.replace(/^\/+|\/+$/g, '')}`.replace(/^\/$/, '');

const escape = (s) => s.replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');

function page(url, script) {
  return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Kinemata has moved</title>
<link rel="canonical" href="${escape(url)}">
<meta http-equiv="refresh" content="0; url=${escape(url)}">
<script>${script}</script>
</head>
<body style="font:16px/1.5 system-ui,sans-serif;padding:2rem;background:#0b0d0c;color:#e8ece9">
<p>Kinemata lives at <a style="color:#c8f560" href="${escape(url)}">${escape(url)}</a>.</p>
</body>
</html>
`;
}

// keep the query string and #anchor of the address the visitor came with
const keep = (url) => `location.replace(${JSON.stringify(url)}+location.search+location.hash);`;

function walk(dir) {
  return readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    return statSync(path).isDirectory() ? walk(path) : [path];
  });
}

rmSync(outDir, {recursive: true, force: true});
let count = 0;
for (const file of walk(buildDir)) {
  if (!file.endsWith('.html')) continue;
  const rel = relative(buildDir, file).split(sep).join('/');
  if (rel === '404.html') continue;
  const path = `/${rel.replace(/(^|\/)index\.html$/, '$1')}`; // a/b/index.html → /a/b/ ; a.html stays
  const url = origin + path;
  const dest = join(outDir, rel);
  mkdirSync(join(dest, '..'), {recursive: true});
  writeFileSync(dest, page(url, keep(url)));
  count += 1;
}

// any other address (old links, typos): same path on the main site, minus this host's base path
const strip = JSON.stringify(prefix);
writeFileSync(
  join(outDir, '404.html'),
  page(
    `${origin}/`,
    `var p=location.pathname,b=${strip};if(b&&p.indexOf(b)===0)p=p.slice(b.length)||'/';` +
      `location.replace(${JSON.stringify(origin)}+p+location.search+location.hash);`,
  ),
);
writeFileSync(join(outDir, '.nojekyll'), '');
console.log(`${count} redirect pages → ${origin} in ${outDir}`);
