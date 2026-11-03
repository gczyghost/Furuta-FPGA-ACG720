"""Durable JSONL operation journal. Each record is flushed before returning."""
from datetime import datetime, timezone
from pathlib import Path
import json
import os
import threading
import uuid
import queue


class Journal:
    def __init__(self, label, directory=None):
        directory = Path(directory or Path(__file__).resolve().parents[1] / 'build/operations')
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = directory / f'{stamp}_{uuid.uuid4().hex[:8]}.jsonl'
        self.file = self.path.open('x', encoding='utf-8')
        self.lock = threading.Lock()
        self.event('session_start', label=label, pid=os.getpid(), cwd=str(Path.cwd()))

    def event(self, event, **fields):
        record = dict(utc=datetime.now(timezone.utc).isoformat(), event=event, **fields)
        self.write_records([record])

    def write_records(self, records):
        with self.lock:
            self.file.writelines(json.dumps(record, ensure_ascii=False) + '\n' for record in records)
            self.file.flush()
            os.fsync(self.file.fileno())

    def close(self):
        self.file.close()


class AsyncJournal:
    """Bounded, ordered journal for the live control loop; durable at close.

    Capture timestamps are assigned by the caller. Disk writes and fsync run
    on a worker so a storage stall cannot delay serial receive or heartbeat.
    Overflow or writer failure raises; callers must STOP on logging failure.
    """
    def __init__(self, label, directory=None, capacity=8192):
        self.journal = Journal(label, directory)
        self.path = self.journal.path
        self.pending = queue.Queue(capacity)
        self.error = None
        self.closed = False
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def _write(self):
        try:
            while True:
                item = self.pending.get()
                if item is None:
                    break
                if isinstance(item,threading.Event):
                    item.set();continue
                records = [item]
                stopping = False
                barrier = None
                while len(records) < 256:
                    try:
                        item = self.pending.get_nowait()
                    except queue.Empty:
                        break
                    if item is None:
                        stopping = True
                        break
                    if isinstance(item,threading.Event):
                        barrier=item;break
                    records.append(item)
                self.journal.write_records(records)
                if barrier is not None:barrier.set()
                if stopping:
                    break
        except Exception as exc:
            self.error = exc
        finally:
            self.journal.close()

    def checkpoint(self,timeout=1.):
        if self.closed or self.error is not None:raise RuntimeError('Journal unavailable')
        barrier=threading.Event();self.pending.put(barrier,timeout=timeout)
        if not barrier.wait(timeout) or self.error is not None:raise RuntimeError('Journal checkpoint failed')

    def event(self, event, **fields):
        if self.closed:
            raise RuntimeError('Journal already closed')
        if self.error is not None:
            raise RuntimeError('Journal writer failed') from self.error
        record = dict(utc=datetime.now(timezone.utc).isoformat(), event=event, **fields)
        try:
            self.pending.put_nowait(record)
        except queue.Full as exc:
            raise RuntimeError('Journal queue full; stop the trial') from exc

    def close(self):
        self.closed = True
        if self.worker.is_alive():
            try:
                self.pending.put(None, timeout=5)
            except queue.Full as exc:
                raise RuntimeError('Journal flush stalled') from exc
            self.worker.join(timeout=10)
        if self.worker.is_alive():
            raise RuntimeError('Journal flush did not finish')
        if self.error is not None:
            raise RuntimeError('Journal writer failed') from self.error
