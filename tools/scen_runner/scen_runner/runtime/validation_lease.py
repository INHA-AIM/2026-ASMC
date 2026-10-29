"""One integrated ego/scene mutator per endpoint in a shared workspace."""
import fcntl
import hashlib
import json
import os
import socket
from pathlib import Path

from scen_runner.config import TOOL_ROOT


class ValidationBusy(RuntimeError):
    pass


class ValidationLease:
    def __init__(self, host, port, directory=None):
        self.endpoint=f'{host}:{int(port)}'
        directory=Path(directory) if directory is not None else Path(os.environ.get('ASMC_VALIDATION_LEASE_DIR',str(TOOL_ROOT/'.runtime')))
        digest=hashlib.sha256(self.endpoint.encode()).hexdigest()[:16]
        self.path=directory/f'validation_{digest}.lock'
        self.fd=None

    @property
    def held(self):
        return self.fd is not None

    def acquire(self):
        if self.held:
            return
        # Docker root and the WSL user use the same bind-mounted directory.
        # The lock persists as a file; the OS releases its ownership on exit.
        if not self.path.parent.exists():
            try:
                self.path.parent.mkdir(parents=True,mode=0o777)
                self.path.parent.chmod(0o777)
            except FileExistsError:
                pass
        fd=os.open(str(self.path),os.O_RDWR|os.O_CREAT,0o666)
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            try:
                owner=json.loads(os.read(fd,4096).decode())
                who=f'{owner.get("host","미확인")} PID {owner.get("pid","미확인")}'
            except (ValueError,OSError):
                who='실행 주체 미확인'
            finally:
                os.close(fd)
            raise ValidationBusy(f'다른 통합 검증이 실행 중입니다 ({who}, {self.endpoint}). '
                                 '현재 창은 ego와 객체를 설정하지 않았습니다. 기존 검증을 종료한 뒤 다시 시작하세요.')
        except BaseException:
            os.close(fd)
            raise
        try:
            # Creation can be affected by umask; only the successful owner
            # changes permissions. A later non-owner cannot remove the lock.
            try:os.fchmod(fd,0o666)
            except PermissionError:
                if self.path.stat().st_mode & 0o666 != 0o666:raise
            payload=json.dumps(dict(endpoint=self.endpoint,pid=os.getpid(),host=socket.gethostname())).encode()
            os.ftruncate(fd,0);os.lseek(fd,0,os.SEEK_SET);os.write(fd,payload)
            self.fd=fd
        except BaseException:
            fcntl.flock(fd,fcntl.LOCK_UN);os.close(fd)
            raise

    def release(self):
        if self.fd is not None:
            fcntl.flock(self.fd,fcntl.LOCK_UN)
            os.close(self.fd)
            self.fd=None
