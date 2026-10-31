"""AST invariants for automatic production edits. Parsing only; never executes candidate code."""
import ast
import subprocess

FILES=('ws/src/navigation/nav_control.launch.py','ws/src/navigation/loc_trust.py','ws/src/navigation/localization_trust_node.py')
KEYS=('footprint','padding','collision_detection','goal_tolerance','xy_tol','yaw_tol','jump_xy','jump_yaw',
      'max_age','fresh','scan_max','T1','T5','inflation_radius','cost_scaling_factor',
      'obstacle_max_range','raytrace_max_range','min_obstacle_height','max_obstacle_height','observation_persistence')


def protected(source):
    tree=ast.parse(source)
    out=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.ClassDef) and node.name in ('Config','TrustConfig','Cfg'):
            out.extend(ast.dump(n,include_attributes=False) for n in node.body if isinstance(n,(ast.Assign,ast.AnnAssign)))
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('X2_FOOTPRINT') for t in node.targets):
            out.append(ast.dump(node,include_attributes=False))
        if isinstance(node,ast.Dict):
            for key,value in zip(node.keys,node.values):
                if isinstance(key,ast.Constant) and isinstance(key.value,str) and any(k in key.value for k in KEYS):
                    out.append(key.value+':'+ast.dump(value,include_attributes=False))
        if isinstance(node,ast.Call) and node.args and isinstance(node.args[0],ast.Constant):
            key=node.args[0].value
            if isinstance(key,str) and any(k in key for k in KEYS):
                out.append(ast.dump(node,include_attributes=False))
    return sorted(out)


def check(repo,base,tip):
    errors=[]
    for path in FILES:
        def blob(ref):
            r=subprocess.run(['git','-C',str(repo),'show',ref+':'+path],capture_output=True,text=True)
            return r.stdout if r.returncode==0 else None
        old,new=blob(base),blob(tip)
        if old==new:continue
        try:
            if old is None or new is None or protected(old)!=protected(new):
                errors.append('protected production safety contract changed: '+path)
        except SyntaxError:
            errors.append('invalid production source: '+path)
    return errors
