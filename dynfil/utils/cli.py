import contextlib
import time

import click


@contextlib.contextmanager
def status(label):
    def update_progress(i, n):
        click.echo('\r' + label.ljust(tw - 9) + '[{:>3}/{:>3}]'.format(i, n), nl=False)

    tw = click.get_terminal_size()[0]
    click.echo(label.ljust(tw - 9) + '[  ...  ]', nl=False)
    tstart = time.time()
    yield update_progress
    tend = time.time()
    tdiff = tend - tstart
    click.echo('\r', nl=False)
    click.echo(click.style(label.ljust(tw - 9) + '[{:>6.2f}s]'.format(tdiff), fg='green'), nl=True)
