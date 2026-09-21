"""Independent MuJoCo backend. Optional policy/DDS packages load on demand."""


def run_experiment(config):
    from .runner import run_experiment as run

    return run(config)


class MujocoBackend:
    @staticmethod
    def run(config):
        return run_experiment(config)

    @staticmethod
    def benchmark(config):
        from .evaluation import run_benchmark

        return run_benchmark(config)
