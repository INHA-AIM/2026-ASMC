#!/usr/bin/env python3
"""Read-only snapshot of the protected source repository (no index refresh, GIT_OPTIONAL_LOCKS=0).

Writes OUT (json) plus OUT.files.tsv (path, size, mtime of every regular file outside build trees) and
OUT.dirs.tsv (path, mtime of every directory). Files and directories are recorded separately so that a
later comparison can state exactly whether only a directory mtime (e.g. a created+deleted cache entry)
or file content/size/mtime changed.
Exit code 3 = branch/HEAD mismatch or dirty tree (BLOCKED_SOURCE_MOVED_OR_DIRTY).
"""
import hashlib, json, os, subprocess, sys, time

sys.dont_write_bytecode = True

REPO = "/home/autonav/2026AIM/2026-ASMC-nogps"
EXPECTED_HEAD = "916e4d9efb38f9066aa4b079df05d16ef011c063"
EXPECTED_BRANCH = "feature/lidar-nogps-main-integration"
OWN_BRANCH_PREFIX = "refs/heads/exp/lidar-obb-quality-temporal-"
ENV = dict(os.environ, GIT_OPTIONAL_LOCKS="0", LC_ALL="C")
SKIP = {".git", "build", "devel", "Testing", "src/lidar+sensorfusion/build", "src/lidar+sensorfusion/devel"}


def git(*args):
    r = subprocess.run(["git", "-C", REPO] + list(args), env=ENV, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return r.returncode, r.stdout.decode("utf-8", "replace"), r.stderr.decode("utf-8", "replace")


def sha_text(s):
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def tree_listing(root):
    files, dirs = [], []
    for dp, dns, fns in os.walk(root):
        rel = os.path.relpath(dp, root)
        keep = []
        for d in sorted(dns):
            r = os.path.normpath(os.path.join(rel, d))
            if r in SKIP:
                continue
            keep.append(d)
        dns[:] = keep
        try:
            dirs.append("%s\t%.6f" % (rel, os.lstat(dp).st_mtime))
        except OSError:
            pass
        for fn in sorted(fns):
            p = os.path.join(dp, fn)
            try:
                st = os.lstat(p)
            except OSError:
                continue
            files.append("%s\t%d\t%.6f" % (os.path.relpath(p, root), st.st_size, st.st_mtime))
    return files, dirs


def main(out):
    _, head, _ = git("rev-parse", "HEAD")
    _, branch, _ = git("branch", "--show-current")
    _, status, _ = git("status", "--porcelain=v1", "--untracked-files=all")
    _, status_ign, _ = git("status", "--porcelain=v1", "--ignored", "--untracked-files=normal")
    _, refs, _ = git("for-each-ref", "--format=%(refname) %(objectname)")
    _, wts, _ = git("worktree", "list", "--porcelain")
    _, stash, _ = git("stash", "list")
    _, gitdir, _ = git("rev-parse", "--absolute-git-dir")
    _, common, _ = git("rev-parse", "--git-common-dir")
    _, lsfiles, _ = git("ls-files", "-s")
    _, remotes, _ = git("remote", "-v")
    rc_diff, _, _ = git("diff", "--quiet", "HEAD")
    gitdir = gitdir.strip()
    common = os.path.normpath(os.path.join(REPO, common.strip()))
    index = os.path.join(gitdir, "index")
    tracked = [l for l in status.splitlines() if l and not l.startswith("??")]
    untracked = [l[3:] for l in status.splitlines() if l.startswith("??")]
    ignored = [l[3:] for l in status_ign.splitlines() if l.startswith("!!")]
    files, dirs = tree_listing(REPO)
    refs_lines = [l for l in refs.splitlines() if not l.startswith(OWN_BRANCH_PREFIX)]
    remote_refs = [l for l in refs.splitlines() if l.startswith("refs/remotes/")]
    fetch_head = os.path.join(common, "FETCH_HEAD")
    snap = {
        "captured_at_epoch": time.time(),
        "captured_at_kst": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        "repo": REPO, "git_dir": gitdir, "git_common_dir": common,
        "head": head.strip(), "branch": branch.strip(),
        "expected_head": EXPECTED_HEAD, "expected_branch": EXPECTED_BRANCH,
        "head_matches": head.strip() == EXPECTED_HEAD, "branch_matches": branch.strip() == EXPECTED_BRANCH,
        "tracked_changes": tracked, "diff_head_quiet_rc": rc_diff,
        "untracked_files": untracked, "ignored_entries": ignored,
        "stash_list": stash.splitlines(),
        "worktree_list_porcelain": wts.splitlines(),
        "remotes": remotes.splitlines(),
        "refs_count_excluding_own_branch": len(refs_lines),
        "refs_sha256_excluding_own_branch": sha_text("\n".join(refs_lines)),
        "remote_refs_sha256": sha_text("\n".join(remote_refs)),
        "own_branch_refs": [l for l in refs.splitlines() if l.startswith(OWN_BRANCH_PREFIX)],
        "fetch_head_mtime": os.stat(fetch_head).st_mtime if os.path.exists(fetch_head) else None,
        "ls_files_stage_sha256": sha_text(lsfiles), "ls_files_count": len(lsfiles.splitlines()),
        "index_sha256": sha_file(index) if os.path.exists(index) else None,
        "index_mtime": os.stat(index).st_mtime if os.path.exists(index) else None,
        "worktree_file_listing_count": len(files),
        "worktree_file_listing_sha256": sha_text("\n".join(files)),
        "worktree_dir_listing_count": len(dirs),
        "worktree_dir_listing_sha256": sha_text("\n".join(dirs)),
        "listing_excludes": sorted(SKIP),
        "listing_note": "files: path,size,mtime; dirs: path,mtime (recorded separately so directory-only mtime changes are distinguishable)",
    }
    with open(out, "w") as f:
        json.dump(snap, f, indent=2, ensure_ascii=False)
    with open(out + ".files.tsv", "w") as f:
        f.write("\n".join(files) + "\n")
    with open(out + ".dirs.tsv", "w") as f:
        f.write("\n".join(dirs) + "\n")
    print(json.dumps({k: snap[k] for k in ("head", "branch", "head_matches", "branch_matches", "tracked_changes",
                                          "untracked_files", "stash_list", "diff_head_quiet_rc",
                                          "worktree_file_listing_count", "worktree_dir_listing_count")}, indent=1))
    clean = snap["head_matches"] and snap["branch_matches"] and not tracked and not untracked and rc_diff == 0
    return 0 if clean else 3


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
