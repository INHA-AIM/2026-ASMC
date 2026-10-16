"""Markdown link positions and GitHub-style anchors, without executing examples."""
from dataclasses import dataclass
import html
import re
from typing import Iterable, Set
from urllib.parse import unquote


@dataclass(frozen=True)
class Link:
    start: int
    end: int
    target: str
    image: bool = False


def prose(text: str, inline_code: bool = True) -> str:
    """Mask code while preserving character positions for surgical URL edits."""
    lines = text.splitlines(keepends=True)
    fence = None
    for i, line in enumerate(lines):
        match = re.match(r'^ {0,3}(`{3,}|~{3,})', line)
        if fence:
            lines[i] = ''.join('\n' if c == '\n' else ' ' for c in line)
            if match and match[1][0] == fence[0] and len(match[1]) >= len(fence):
                fence = None
        elif match:
            fence = match[1]
            lines[i] = ''.join('\n' if c == '\n' else ' ' for c in line)
    result = ''.join(lines)
    if inline_code:
        result = re.sub(r'(`+)(?!`)(.+?)(?<!`)\1(?!`)',
                        lambda m: ''.join('\n' if c == '\n' else ' ' for c in m[0]),
                        result, flags=re.S)
    return result


def _destination(text: str, start: int):
    if start >= len(text):
        return None
    if text[start] == '<':
        end = text.find('>', start + 1)
        return (start + 1, end) if end >= 0 else None
    end, depth = start, 0
    while end < len(text):
        char = text[end]
        if char == '\\' and end + 1 < len(text):
            end += 2
            continue
        if char == '(':
            depth += 1
        elif char == ')':
            if depth == 0:
                break
            depth -= 1
        elif char.isspace() and depth == 0:
            break
        end += 1
    return start, end


def links(text: str) -> Iterable[Link]:
    visible = prose(text)
    seen = set()
    for pattern in (r'!?\[[^\]\n]*\]\(\s*', r'^ {0,3}\[[^\]\n]+\]:\s*'):
        for match in re.finditer(pattern, visible, flags=re.M):
            span = _destination(text, match.end())
            if not span or span in seen:
                continue
            start, end = span
            if start == end:
                continue
            seen.add(span)
            yield Link(start, end, text[start:end], match[0].startswith('!'))
    for match in re.finditer(r'<(?:a|img)\b[^>]*?\b(?:href|src)\s*=\s*([\'"])(.*?)\1', visible, flags=re.I):
        start, end = match.span(2)
        if (start, end) not in seen:
            yield Link(start, end, text[start:end], match[0].lower().startswith('<img'))


def local_target(target: str) -> bool:
    return not re.match(r'^(?:[a-z][a-z0-9+.-]*:|//)', target, flags=re.I)


def split_target(target: str):
    path, sep, fragment = target.partition('#')
    path, query_sep, query = path.partition('?')
    return unquote(path), ('?' + query if query_sep else '') + ('#' + fragment if sep else ''), unquote(fragment)


def anchors(text: str) -> Set[str]:
    visible = prose(text, inline_code=False)
    result = set(re.findall(r'<a\b[^>]*\b(?:id|name)=[\'"]([^\'"]+)', visible, flags=re.I))
    counts = {}
    lines = visible.splitlines()
    for i, line in enumerate(lines):
        match = re.match(r'^ {0,3}#{1,6}\s+(.+?)(?:\s+#+\s*)?$', line)
        title = match[1] if match else None
        if title is None and i + 1 < len(lines) and line.strip() and re.match(r'^ {0,3}(?:=+|-+)\s*$', lines[i + 1]):
            title = line.strip()
        if title is None:
            continue
        title = re.sub(r'!?\[([^\]]*)\]\([^)]*\)', r'\1', title)
        title = re.sub(r'<[^>]*>', '', title)
        title = html.unescape(title).replace('`', '')
        slug = re.sub(r'[^\w\- ]', '', title.lower()).replace(' ', '-')
        count = counts.get(slug, 0)
        counts[slug] = count + 1
        result.add(slug + ('-' + str(count) if count else ''))
    return result


def without_link_destinations(text: str) -> str:
    result = text
    for link in sorted(links(text), key=lambda x: x.start, reverse=True):
        result = result[:link.start] + '<LINK>' + result[link.end:]
    return result
