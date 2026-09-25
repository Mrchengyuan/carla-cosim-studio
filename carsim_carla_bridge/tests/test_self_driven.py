"""Offline check of the "CarSim driver" mode wrapper (no CarSim, no CARLA).

    python tests/test_self_driven.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "python_carsim_env"))


class FakeSolver:
    """VS solver whose run declares no imports (CarSim drives itself)."""
    def __init__(self, n_import):
        self.n_import, self.imports_seen = n_import, []

    def read_configuration(self, path):
        return {"n_import": self.n_import, "n_export": 2, "t_start": 0.0, "t_stop": 1.0, "t_step": 0.001}

    def copy_export_vars_into(self, arr, n, return_list=False):
        pass

    def integrate_io_inplace(self, t, imp, exp, n):
        self.imports_seen.append(list(imp))
        return 0

    def terminate_run(self, t):
        pass


def make(n_import):
    from carsim_env import CarSimEnv
    from session import self_driven
    env = CarSimEnv.__new__(CarSimEnv)      # skip DLL loading
    env.sim_path, env.solver = "x.sim", FakeSolver(n_import)
    env.initialized, env.done, env.import_vars, env.export_vars = False, True, [], []
    return self_driven(env)


def main():
    ok = True
    env = make(0)
    env.reset()                             # stock CarSimEnv refuses n_import == 0
    env.control_step(None, 5)
    ok &= env.declared_imports == 0 and len(env.solver.imports_seen) == 5
    print(("PASS" if ok else "FAIL"), "run without imports steps, declared_imports =", env.declared_imports)

    env = make(3)
    env.reset()
    env.import_np[:] = [7.0, 8.0, 9.0]      # whatever is there is left alone
    env.control_step(None, 1)
    good = env.solver.imports_seen[-1] == [7.0, 8.0, 9.0] and env.declared_imports == 3
    print(("PASS" if good else "FAIL"), "None action never writes the imports")
    ok &= good
    env.control_step([0.5, 0.0, 10.0], 1)
    good = env.solver.imports_seen[-1] == [0.5, 0.0, 10.0]
    print(("PASS" if good else "FAIL"), "explicit actions still written")
    ok &= good
    print("ALL SELF-DRIVEN TESTS PASSED" if ok else "SOME TESTS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
