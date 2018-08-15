/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "cost_function.h"

namespace ntk
{

double CostFunction :: outputNorm(const std::vector<double>& input) const
{
  std::vector<double> output(m_output_dim);
  evaluate(input, output);
  double err = 0;
  foreach_idx(i, output)
  {
    err += output[i]*output[i];
  }
  err = sqrt(err);
  return err;
}

}
