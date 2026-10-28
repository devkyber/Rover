"""Remux copied H.264 fragments into a convenient playback file, no re-encoding."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import av
import argparse
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[3]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path, help='New output directory; must not exist')
args=parser.parse_args()
HERE=args.output.resolve();HERE.mkdir(parents=True,exist_ok=False)
original=ROOT/'logs/video/practice_e2c0c0d3_19700101T000717.904946Z_b94e84c0'
segments=sorted(original.glob('video_*.ts'))
listing=HERE/'segments.ffconcat'
listing.write_text('ffconcat version 1.0\n'+''.join("file '"+p.as_posix()+"'\n" for p in segments),encoding='utf-8')
output=HERE/'practice_e2c0c0d3.mp4'
command=[imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-nostdin','-n','-f','concat','-safe','0',
    '-i',str(listing),'-map','0:v:0','-c:v','copy','-an','-movflags','+faststart',str(output)]
result=subprocess.run(command,capture_output=True,timeout=45)
(HERE/'remux.log').write_bytes(result.stdout+result.stderr)
result.check_returncode()
with av.open(str(output)) as container:
    stream=container.streams.video[0]
    playback=dict(duration_s=container.duration/av.time_base if container.duration is not None else None,
        codec=stream.codec_context.name,width=stream.width,height=stream.height,frames=stream.frames)
    first=next(container.decode(video=0))
    first.to_image().save(HERE/'video_first_frame.jpg',quality=90)
rows=[json.loads(line) for line in (original/'frames.jsonl').read_text().splitlines()]
frames=[row for row in rows if row['kind']=='frame']
run_first_mono=67.807517472
cut_mono=475.427694
record=dict(original_directory=original.relative_to(ROOT).as_posix(),
    run_id='e2c0c0d377de495d8d423acefa0d2710',
    original_segments=len(segments),journal_events=dict(Counter(row['kind'] for row in rows)),
    first_frame_monotonic_s=frames[0]['mono'],last_frame_monotonic_s=frames[-1]['mono'],
    wall_span_s=frames[-1]['mono']-frames[0]['mono'],
    graph_elapsed_start_s=frames[0]['mono']-run_first_mono,
    graph_elapsed_end_s=frames[-1]['mono']-run_first_mono,
    inferred_connection_cut_after_first_frame_s=cut_mono-frames[0]['mono'],
    playback=playback,output=str(output),
    output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
    bytes=output.stat().st_size,reencoded=False,
    continuity='Concatenated existing fragments. Missing intervals are not reconstructed; playback time is not wall time.',
    missing_summary=True,short_run_218d_matching_video_found=False)
(HERE/'playback_receipt.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
print(json.dumps(record,indent=2))
