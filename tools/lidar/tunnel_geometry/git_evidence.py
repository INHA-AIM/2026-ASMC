"""Read-only Git evidence without changing user/container Git configuration."""
import os
from pathlib import Path
import subprocess
import tempfile


def git_output(repo, arguments):
    # Ubuntu's Git 2.25 backport ignores command-scope safe.directory. Its XDG
    # config is supported: use a private temporary one, scoped to this process.
    # Do not broaden trust to '*' or mutate the user's global configuration.
    with tempfile.TemporaryDirectory(prefix="asmc-l1b4-git-") as directory:
        config = Path(directory) / "git" / "config"
        config.parent.mkdir()
        subprocess.run(["git", "config", "--file", str(config), "safe.directory", str(repo)],
                       check=True)
        environment = dict(os.environ, XDG_CONFIG_HOME=directory)
        return subprocess.check_output(["git"] + list(arguments), cwd=repo,
                                       env=environment, text=True)
