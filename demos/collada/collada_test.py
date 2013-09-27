from adolphus.interface import Experiment

ex = Experiment(zoom=False)
ex.execute('loadmodel %s' % 'scene.yaml')
ex.execute('loadconfig %s' % '')

ex.start()
