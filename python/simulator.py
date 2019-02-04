"""
BSD 3-Clause License

Copyright (c) 2019, HJ Reachability Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author(s): Ellis Ratner ( eratner@eecs.berkeley.edu )
"""
import yaml
from abc import ABCMeta, abstractmethod
import numpy as np
from sympy import symbols, Matrix
from sympy.parsing.sympy_parser import parse_expr
from sympy.utilities.lambdify import lambdify
from visual import dubins_car


class Integrator:
    __metaclass__ = ABCMeta

    @abstractmethod
    def integrate(self):
        pass


class RungeKutta(Integrator):
    def __init__(self):
        Integrator.__init__(self)

    def integrate(self, t_init, x_init, u_init, f, step_size):
        args = [t_init] + x_init.tolist() + u_init.tolist()
        k1 = step_size*f(*args).flatten()

        args = [t_init + 0.5*step_size] + (x_init + 0.5*k1).tolist() + u_init.tolist()
        k2 = step_size*f(*args).flatten()

        args = [t_init + 0.5*step_size] + (x_init + 0.5*k2).tolist() + u_init.tolist()
        k3 = step_size*f(*args).flatten()

        args = [t_init + 0.5*step_size] + (x_init + k3).tolist() + u_init.tolist()
        k4 = step_size*f(*args).flatten()

        return x_init + (k1 + 2.*k2 + 2.*k3 + k4)/6.


class DynamicSystem:
    def __init__(self, config):
        self.ident = config['ident']

        # Parse the dynamics.
        t = symbols('t')
        x = symbols(config['dynamics']['state'])
        u = symbols(config['dynamics']['input'])

        self.state_dim = len(x)
        self.input_dim = len(u)

        dynamics = []
        for d in config['dynamics']['dxdt']:
            dynamics.append(parse_expr(d))

        dxdt = Matrix(dynamics).transpose()
        args = (t,) + x + u
        # Dynamics function has the form dxdt = f(t, x_1, x_2, ..., u_1, u_2, ...).
        self.f = lambdify(args, dxdt, modules=[{'ImmutableMatrix': np.array}, 'numpy'])

        print("Loaded system {} with state_dim == {}, input_dim == {}".format(self.ident,
                                                                              self.state_dim,
                                                                              self.input_dim))

        # TODO allow the config file to specify what integrator is being used.
        self.integrator = RungeKutta()

    def step(self, dt, curr_t, curr_state, curr_input):
        return self.integrator.integrate(curr_t, curr_state, curr_input, self.f, dt)


class Simulator:
    def __init__(self, config_file):
        with open(config_file, 'r') as f:
            self.config = yaml.load(f)

        # Load systems to simulate. 
        self.systems = {}
        self.systems_state = {}

        if 'systems' in self.config:
            for sys_config in self.config['systems']:
                system = DynamicSystem(sys_config)
                
                self.systems[system.ident] = system
                self.systems_state[system.ident] = np.array(sys_config['initial_state'])

    def step(self, inputs, curr_t, dt):
        for ident, sys in self.systems.iteritems():
            # Get the current state of the system. 
            curr_state = self.systems_state[ident]

            # Get current input if there is one; otherwise, assume zero input.
            curr_input = np.zeros(sys.input_dim)
            if ident in inputs:
                curr_input = inputs[ident]

            # Get the next state of the system.
            next_state = sys.step(dt, curr_t, curr_state, curr_input)
            self.systems_state[ident] = next_state

    def get_markers_ros(self):
        # TODO fix this.
        markers = []

        for ident, state in self.systems_state.iteritems():
            markers += dubins_car(ident, state)

        return markers
