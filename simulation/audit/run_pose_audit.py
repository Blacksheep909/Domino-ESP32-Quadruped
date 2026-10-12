"""Reproducible, offline CAD -> firmware -> pulse -> planted-foot audit.

Requires NumPy, Node and the local MinGW SIL compiler. No robot connection.
Outputs are predictions conditional on CAD geometry, calibration and servo gain;
they are not measurements of physical joints, foot positions or PWM timing.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CHANNELS = [0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15]
OFFSETS = [-5, 5, -3, 7, -.5, 1, 3, 8.5, -9, 0, -6, -3]
PROFILES = ['default', 'recordingCandidate', 'leftHipsInvertedCandidate']
CAD = ROOT / 'simulation/urdf/generated/Domino_URDF_Parts_Combined_Final_description'


def rotation(rpy):
    r, p, y = np.radians(rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def rotate2(point, centre, radians):
    c, s = math.cos(radians), math.sin(radians)
    return centre + np.array([[c, -s], [s, c]]) @ (point-centre)


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def triangle_vertex(a, b, ra, rb, reference_sign):
    # Independent triangle-angle construction, not the renderer's circle
    # intersection/scaling function. An impossible closure raises an error.
    d = float(np.linalg.norm(b-a))
    cosine = (ra*ra+d*d-rb*rb)/(2*ra*d) if d > 1e-9 else 2
    if abs(cosine) > 1+1e-7:
        raise ValueError('unreachable linkage closure')
    angle = math.atan2(*(b-a)[::-1]) + math.copysign(math.acos(np.clip(cosine, -1, 1)), reference_sign)
    return a + ra*np.array([math.cos(angle), math.sin(angle)])


def body_foot(leg, angles, reference, origin):
    p = {k: np.array(v, dtype=float)*1000 for k, v in leg['points'].items()}
    xz = {k: v[[0, 2]] for k, v in p.items()}
    q = {j: (angles[ch]-reference[ch])/leg['directions'][j] for j, ch in leg['channels'].items()}
    offset = leg.get('neutralLinkageOffsetDeg', {})
    # Names here identify the actual connected parts, irrespective of the
    # inconsistent parent/child names in the exported URDF closure joints.
    lower = xz['lower_drive']
    upper = xz['upper_drive']
    passive = rotate2(xz['lower_passive'], lower, math.radians(-q['lower']+offset.get('lower', 0)))
    pin_lower = rotate2(xz['lower_closure_diagonal'], lower, math.radians(-q['lower']+offset.get('lower', 0)))
    pin_upper = rotate2(xz['upper_closure_driver'], upper, math.radians(-q['upper']+offset.get('upper', 0)))
    a0, b0, c0 = xz['lower_passive'], xz['upper_closure_driver'], xz['upper_closure_coupler']
    coupler_upper = triangle_vertex(passive, pin_upper, np.linalg.norm(c0-a0), np.linalg.norm(c0-b0), cross(b0-a0, c0-a0))
    coupler_angle = math.atan2(*(coupler_upper-passive)[::-1])-math.atan2(*(c0-a0)[::-1])
    coupler_lower = rotate2(xz['lower_coupler'], a0, coupler_angle) + passive-a0
    a0, b0, c0 = xz['lower_coupler'], xz['lower_closure_diagonal'], xz['lower_closure_driver']
    pin_diagonal = triangle_vertex(coupler_lower, pin_lower, np.linalg.norm(c0-a0), np.linalg.norm(c0-b0), cross(b0-a0, c0-a0))
    closure_angle = math.atan2(*(pin_diagonal-pin_lower)[::-1])-math.atan2(*(c0-b0)[::-1])
    foot_xz = rotate2(xz['foot_tip'], b0, closure_angle) + pin_lower-b0
    hip = p['hip_origin']
    foot = np.array([foot_xz[0], p['foot_tip'][1], foot_xz[1]])
    return hip + rotation([-leg['shoulderSign']*q['shoulder'], 0, 0]) @ (foot-hip) - origin


def rigid_fit(body_points, world_points):
    # Infer the chassis pose solely from commanded linkage endpoints and the
    # planted-foot constraint; no requested body attitude or assist forces.
    a, b = body_points.mean(axis=0), world_points.mean(axis=0)
    u, _, vt = np.linalg.svd((body_points-a).T @ (world_points-b))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rot = vt.T @ correction @ u.T
    translation = b-rot@a
    residual = np.linalg.norm((rot@body_points.T).T+translation-world_points, axis=1)
    return rot, translation, residual


def mesh_vertices(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84+50*count:
        raise ValueError(f'Expected binary STL: {path}')
    dtype = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.unique(np.frombuffer(raw, offset=84, count=count, dtype=dtype)['v'].reshape(-1, 3), axis=0)


def pin_circle(vertices, expected):
    # Locate a triangulated pin bore by radius around an approximate seed, then
    # independently fit its centre. Radius bins keep the adjacent outer wall out.
    xz = np.unique(vertices[:, [0, 2]], axis=0)
    distances = np.linalg.norm(xz-expected, axis=1)
    candidates = []
    for radius in np.arange(1.5, 6.05, .05):
        points = xz[np.abs(distances-radius) < .018]
        if len(points) < 12:
            continue
        a = np.column_stack([2*points[:, 0], 2*points[:, 1], np.ones(len(points))])
        centre = np.linalg.lstsq(a, (points*points).sum(axis=1), rcond=None)[0][:2]
        radii = np.linalg.norm(points-centre, axis=1)
        candidates.append(dict(centreMm=centre.tolist(), radiusMm=float(radii.mean()),
                               fitRmsMm=float(np.std(radii)), seedErrorMm=float(np.linalg.norm(centre-expected)), vertices=len(points)))
    return sorted(candidates, key=lambda v: (-v['vertices'], v['fitRmsMm']))[:1]


def cad_audit(config):
    build_cad = ROOT/'Domino_BUILD_Parts/URDF/Domino_URDF_Parts_Combined_Final_description'
    hashes = []
    for p in sorted(CAD.rglob('*')):
        if p.suffix.lower() not in ['.stl', '.urdf']:
            continue
        other = build_cad/p.relative_to(CAD)
        hashes.append(dict(file=str(p.relative_to(CAD)), sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                           matchesBuildExport=other.exists() and p.read_bytes() == other.read_bytes()))
    base = mesh_vertices(CAD/'meshes/base_link.stl')
    pins = []
    pin_parts = {'lower_drive': 'lower_driver', 'upper_drive': 'upper_driver',
                 'upper_closure_driver': 'upper_driver', 'lower_closure_diagonal': 'lower_driver',
                 'lower_closure_driver': 'lower_closure', 'upper_closure_coupler': 'coupler',
                 'lower_coupler': 'coupler'}
    for leg in config['legs']:
        for point, part in pin_parts.items():
            mesh = leg['meshes'][part]
            vertices = mesh_vertices(CAD/f'meshes/{mesh}.stl')
            expected = (np.array(leg['points'][point])-np.array(leg['sourceTranslation']))[[0, 2]]*1000
            fit = pin_circle(vertices, expected)
            pins.append(dict(leg=leg['label'], point=point, mesh=mesh, expectedMm=expected.tolist(), fit=fit))
    tree = ET.parse(CAD/'urdf/Domino_URDF_Parts_Combined_Final.urdf').getroot()
    hips = [dict(name=j.attrib['name'], origin=j.find('origin').attrib['xyz'], axis=j.find('axis').attrib['xyz'])
            for j in tree.findall('joint') if j.find('parent').attrib['link'] == 'base_link']
    # STL vertices use assembly coordinates. Exported visual origins move them
    # into each link frame, so undo that translation to inspect every URDF pin.
    links = {link.attrib['name']: link for link in tree.findall('link')}
    connections = [('lower_driver', 'lower_closure', 'lower_closure_diagonal'),
                   ('lower_driver', 'coupler', 'lower_passive'),
                   ('upper_driver', 'upper_closure', 'upper_closure_driver'),
                   ('coupler', 'upper_closure', 'upper_closure_coupler'),
                   ('coupler', 'lower_diagonal', 'lower_coupler'),
                   ('lower_diagonal', 'lower_closure', 'lower_closure_driver')]
    urdf_pins = []
    for leg in [leg for leg in config['legs'] if leg['label'] in ['BL', 'BR']]:
        for parent_part, child_part, point in connections:
            parent, child = leg['meshes'][parent_part], leg['meshes'][child_part]
            joint = next(j for j in tree.findall('joint') if j.find('parent').attrib['link'] == parent and j.find('child').attrib['link'] == child)
            visual_origin = links[parent].find('visual/origin')
            assert visual_origin.attrib.get('rpy', '0 0 0') == '0 0 0'
            xyz = np.array([float(v) for v in joint.find('origin').attrib['xyz'].split()])
            xyz -= np.array([float(v) for v in visual_origin.attrib['xyz'].split()])
            expected = np.array(leg['points'][point])*1000
            urdf_pins.append(dict(leg=leg['label'], joint=joint.attrib['name'], pin=point,
                urdfAxisPointMm=(xyz*1000).tolist(), meshAxisPointMm=expected.tolist(),
                xzAxisErrorMm=float(np.linalg.norm((xyz*1000-expected)[[0, 2]]))))
    return dict(files=hashes, hipJoints=hips, pins=pins, urdfPins=urdf_pins, baseBoundsMm=[base.min(axis=0).tolist(), base.max(axis=0).tolist()],
                bodyOriginMm=(np.array(config['assemblyOrigin'])*1000).tolist())


def compile_probe(output, compiler):
    sources = ['simulation/audit/pose_probe.cpp', 'src/crsf.cpp', 'src/gait_profile.cpp',
               'src/manual_control_guard.cpp', 'src/power_fault_guard.cpp', 'src/power_monitor_math.cpp',
               'src/leg_controller.cpp', 'src/servo_calibration.cpp', 'src/ik.cpp',
               'simulation/sil/src/arduino_sim.cpp', 'simulation/sil/src/pwm_sim.cpp', 'simulation/sil/src/imu_sim.cpp']
    args = [str(compiler), '-std=c++11', '-O2', '-static', '-DDOMINO_SIL=1',
            '-Isimulation/sil/include', '-Isrc', '-Ilib/Ramp/src', *sources, '-o', str(output)]
    env = dict(os.environ, PATH=str(compiler.parent)+os.pathsep+os.environ['PATH'])
    subprocess.run(args, cwd=ROOT, env=env, check=True, capture_output=True, text=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'tmp/full-pose-audit')
    parser.add_argument('--session', type=Path)
    parser.add_argument('--node', type=Path, default=Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe')
    parser.add_argument('--compiler', type=Path, default=Path.home()/'.platformio/packages/toolchain-gccmingw32/bin/g++.exe')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    export = 'import {assemblyOrigin,legs,standServoReference} from "./simulation/standalone/web/src/domino-config.js"; console.log(JSON.stringify({assemblyOrigin,legs,standServoReference}));'
    config = json.loads(subprocess.check_output([str(args.node), '--input-type=module', '-e', export], cwd=ROOT, text=True))
    legs, reference, origin = config['legs'], np.array(config['standServoReference']), np.array(config['assemblyOrigin'])*1000
    anchors = np.array([body_foot(leg, reference, reference, origin) for leg in legs])
    probe = args.output/'pose_probe.exe'
    compile_probe(probe, args.compiler)
    cases = []
    for profile in range(len(PROFILES)):
        for height in [220, 230, 250, 270, 280]:
            for roll in range(-20, 21, 5):
                for pitch in range(-10, 11, 2):
                    for yaw in range(-25, 26, 5):
                        if (roll/20)**2+(pitch/10)**2+(yaw/25)**2 <= 1+1e-8:
                            cases.append([profile, height, roll, pitch, yaw, 0, 0])
        for body_x, body_y in [(-15, 0), (15, 0), (0, -12), (0, 12)]:
            cases.append([profile, 250, 0, 0, 0, body_x, body_y])
    session = json.loads(args.session.read_text(encoding='utf-8-sig'))['session']['samples'] if args.session else []
    session_start = len(cases)
    for sample in session:
        p = sample['expectedBody']
        cases.append([1, p['heightMm'], p['rollDeg'], p['pitchDeg'], p['yawDeg'], 0, 0])
    stdin = ''.join(' '.join(map(str, case))+'\n' for case in cases)
    outputs = [json.loads(line) for line in subprocess.check_output([str(probe)], input=stdin, text=True, cwd=ROOT).splitlines()]
    if len(outputs) != len(cases):
        raise RuntimeError('Missing probe outputs')
    rows = []
    for index, (case, output) in enumerate(zip(cases, outputs)):
        profile, height, roll, pitch, yaw, bx, by = case
        target_rotation = rotation([roll, pitch, yaw])
        target_translation = np.array([bx, by, height-280])
        raw = np.array(output['angles'])
        command = np.array(output['model'])
        # Adafruit v3.0.3 uses prescale 131 at 27 MHz / 50 Hz, then truncates
        # microseconds to a 12-bit tick. SIL historically omits that second step.
        pulse_angles = np.maximum(0, np.floor(np.array(output['pulses'])*27/132)*132/27-500)*270/2000
        if profile >= 1:
            pulse_angles[CHANNELS] -= OFFSETS
        if profile == 2:
            pulse_angles[[0, 14]] = 2*reference[[0, 14]]-pulse_angles[[0, 14]]
        row = dict(case=case, hardClips=output['hardClips'])
        angle_sources = [('command', command), ('pca9685', pulse_angles)]
        if index >= session_start:
            recorded = reference.copy()
            recorded[CHANNELS] = np.array(session[index-session_start]['expectedJointAnglesDeg'])-OFFSETS
            angle_sources.append(('recordedCommand', recorded))
        for name, angles in angle_sources:
            try:
                feet = np.array([body_foot(leg, angles, reference, origin) for leg in legs])
                errors = np.linalg.norm((target_rotation@feet.T).T+target_translation-anchors, axis=1)
                fit_rotation, fit_translation, residual = rigid_fit(feet, anchors)
                angle_error = math.degrees(math.acos(np.clip((np.trace(target_rotation.T@fit_rotation)-1)/2, -1, 1)))
                row[name] = dict(maxFootErrorMm=float(errors.max()), fittedTranslationMm=fit_translation.tolist(),
                                 centreErrorMm=float(np.linalg.norm(fit_translation-target_translation)),
                                 attitudeErrorDeg=angle_error, fitResidualMm=float(residual.max()))
            except ValueError as error:
                row[name] = dict(error=str(error))
        if index >= session_start:
            sample = session[index-session_start]
            row['elapsedMs'] = sample['elapsedMs']
            row['sagittalRecordedErrorDeg'] = max(abs(sample['expectedJointAnglesDeg'][j]-raw[ch])
                for j, ch in enumerate(CHANNELS) if ch not in [0, 3, 9, 14])
        rows.append(row)
    summary = dict(cad=cad_audit(config), envelopeCases=session_start, sessionCases=len(session),
                   assumptions=['CAD geometry describes hardware', 'candidate offsets inferred from old recording, not exported active profile',
                                'third profile adds owner-confirmed left hip inversion; other offsets/limits assumed unchanged',
                                '270 degrees over 500-2500 us; PCA clock exactly 27 MHz', 'no servo lag, flex, backlash or encoder feedback'])
    for profile, name in enumerate(PROFILES):
        selected = [r for r in rows[:session_start] if r['case'][0] == profile]
        stats = dict(count=len(selected), hardClippedCases=sum(r['hardClips'] > 0 for r in selected))
        for kind in ['command', 'pca9685']:
            valid = [r for r in selected if 'error' not in r[kind]]
            values = [r[kind]['maxFootErrorMm'] for r in valid]
            stats[kind] = dict(unreachable=len(selected)-len(valid), maxFootErrorMm=max(values),
                               p95FootErrorMm=float(np.percentile(values, 95)), over5mm=sum(v > 5 for v in values),
                               worst=sorted(valid, key=lambda r: r[kind]['maxFootErrorMm'], reverse=True)[:8])
        isolated_pitch = [r for r in selected if r['case'][2] == r['case'][4] == 0 and r['case'][3] != 0]
        stats['isolatedPitch'] = dict(count=len(isolated_pitch), hardClippedCases=sum(r['hardClips'] > 0 for r in isolated_pitch),
            maxFootErrorMm=max(r['command']['maxFootErrorMm'] for r in isolated_pitch),
            maxCentreErrorMm=max(r['command']['centreErrorMm'] for r in isolated_pitch),
            maxPulseFootErrorMm=max(r['pca9685']['maxFootErrorMm'] for r in isolated_pitch))
        summary[name] = stats
    post = [r for r in rows[session_start:] if r['elapsedMs'] >= 4300]
    if post:
        summary['recordedSagittalMaxDifferenceDeg'] = max(r['sagittalRecordedErrorDeg'] for r in post)
        isolated = [r for r in post if r['case'][2] == 0 and abs(r['case'][3]) > 1]
        summary['recording'] = dict(postTransitionRows=len(post), isolatedPitchRows=len(isolated),
            postTransitionHardClippedRows=sum(r['hardClips'] > 0 for r in post),
            maxRecordedPitchFootErrorMm=max(r['recordedCommand']['maxFootErrorMm'] for r in isolated),
            maxRecordedPitchCentreErrorMm=max(r['recordedCommand']['centreErrorMm'] for r in isolated))
        summary['source'] = dict(session=str(args.session), sha256=hashlib.sha256(args.session.read_bytes()).hexdigest())
    # Fail if the essential independent geometry checks stop holding. These do
    # not certify a physical calibration or hide clipped candidate-profile poses.
    assert all(f['matchesBuildExport'] for f in summary['cad']['files']), 'Build and rendered CAD diverged'
    assert all(p['fit'] and p['fit'][0]['seedErrorMm'] < .01 for p in summary['cad']['pins']), 'STL pin verification failed'
    assert summary['default']['command']['unreachable'] == 0
    assert summary['default']['command']['maxFootErrorMm'] < 1, 'Default CAD/firmware endpoint disagreement'
    assert all(summary[name]['isolatedPitch']['maxFootErrorMm'] < 1 for name in PROFILES)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    (args.output/'poses.json').write_text(json.dumps(rows), encoding='utf-8')
    print(json.dumps({k: v for k, v in summary.items() if k not in ['cad', *PROFILES]}, indent=2))
    for name in PROFILES:
        print(name, json.dumps({k: ({a:b for a,b in v.items() if a != 'worst'} if isinstance(v, dict) else v)
                                for k,v in summary[name].items()}))
    print('Artifacts:', args.output)


if __name__ == '__main__':
    main()
