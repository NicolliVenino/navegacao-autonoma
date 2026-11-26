from setuptools import find_packages, setup

package_name = 'cg_solver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolli Venino Santana',
    maintainer_email='nicolli.santana@sou.inteli.edu.br',
    description='Navegação autônoma no labirinto do Culling Games (A* + mapeamento por DFS)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parte1 = cg_solver.parte1:main',
            'parte2 = cg_solver.parte2:main',
        ],
    },
)
