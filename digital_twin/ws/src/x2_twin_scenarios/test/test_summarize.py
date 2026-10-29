import json

from x2_twin_scenarios.analysis.summarize import summarize


def _run(tmp_path, name, out_status, out_err, back_status, back_err):
    d = tmp_path / name
    d.mkdir()
    checks = [
        dict(name='goal_out_succeeded', ok=out_status == 4, detail=out_status, hard=False),
        dict(name='goal_out_pose', ok=False, detail=dict(xy_error=out_err), hard=False),
        dict(name='goal_back_succeeded', ok=back_status == 4, detail=back_status, hard=False),
        dict(name='goal_back_pose', ok=False, detail=dict(xy_error=back_err), hard=False),
    ]
    (d / 'results.json').write_text(json.dumps(dict(checks=checks)))


def test_summarize_reads_goal_outcomes(tmp_path):
    _run(tmp_path, 'r01', 4, 0.05, 6, 1.7)
    _run(tmp_path, 'r02', 6, 0.23, 4, 0.08)
    rows = summarize(tmp_path, 'r')
    assert [r[0] for r in rows] == ['r01', 'r02']
    assert rows[0][1:] == (4, 0.05, 6, 1.7)
    assert sum(r[1] == 4 for r in rows) == 1 and sum(r[3] == 4 for r in rows) == 1


def test_missing_results_file_is_reported_not_raised(tmp_path):
    (tmp_path / 'r99').mkdir()
    assert summarize(tmp_path, 'r')[0] == ('r99', None, None, None, None)
