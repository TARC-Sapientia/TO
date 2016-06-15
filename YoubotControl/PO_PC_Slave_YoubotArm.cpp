/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */


#include "PO_PC_Slave_YoubotArm.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace std;

PO_PC_Slave_YoubotArm::PO_PC_Slave_YoubotArm(string logPath, string logName, Slave_PO_PC_Type po_pc_Type, double T, double max_force, double max_velocity)
    :
      m_PO_PC_Type(po_pc_Type),
      m_T(T),
      m_max_force(max_force),
      m_max_velocity(max_velocity),
      m_P_S_treshold(0.0),
      m_f_S_treshold(0.0),
      m_TI_S(10),
      m_delta_S(0.01)
{
    Reset_PO_PC();
    
    m_PO_PC_Logger.reset(new Utils::Log(logPath, logName, false));
}

PO_PC_Slave_YoubotArm::~PO_PC_Slave_YoubotArm()
{
}

void PO_PC_Slave_YoubotArm::Reset_PO_PC()
{
    m_E_local_OUT = 0;
    m_E_local_IN = 0;
    m_PC_Energy = 0;
    m_PC_signal = 0;
    m_PC_signal_old = 0;
    m_v_old = 0;
    m_f_old = 0;
    m_z = 0;
    m_PC_signal_F = 0;
    m_ControlSignal = 0;
    m_E_Remote_del_old = 0;
}

double PO_PC_Slave_YoubotArm::DeadZone(double value, double DZ)
{
    double output;

    if (abs(value) <  DZ)
    {
        output = 0;
	}
	else
    {
        if (value > 0)
        {
            output = value - DZ;
        }
        else // value < 0
        {
            output = value + DZ;
        }
    }

	return output; 
}

double PO_PC_Slave_YoubotArm::Sign(double value)
{
	if (value == 0)
	{
        return 1.0;
	}
	else if (value > 0)
	{
		return 1.0;
	}
	else // value < 0
	{
		return -1.0;
	}
}

double PO_PC_Slave_YoubotArm::GetControlSignal(double f_local, double v_local, double f_remote_del, double v_remote_del, double E_remote_del, double& E_local_toSend, double p_local, double p_remote_del)
{
    double control_signal;

    Passivity_Observer(f_local, v_remote_del, m_P_S_treshold);

    if (E_remote_del < m_E_Remote_del_old)
    {
        E_remote_del = m_E_Remote_del_old;
    }
    else
    {
        m_E_Remote_del_old = E_remote_del;
    }

    control_signal = Passivity_Controller(f_local, v_local, f_remote_del, v_remote_del, E_remote_del);

    m_PC_signal_old = m_PC_signal;
    m_v_old = v_remote_del;
    m_f_old = f_local;

    stringstream ss;
    ss << fixed << setprecision(10)
       << " f_local =" << f_local
       << " v_local =" << v_local
       << " f_remote_del =" << f_remote_del
       << " v_remote_del =" << v_remote_del
       << " E_IN_M =" << E_remote_del
       << " m_ControlSignal =" << m_ControlSignal
       << " E_IN_S =" <<  m_E_local_IN
       << " E_OUT_S =" << m_E_local_OUT
       << " m_PC_signal = " << m_PC_signal_F
       << " m_PC_Energy = " << m_PC_Energy
       << " m_z = " << m_z
       << " p_local = " << p_local
       <<  " p_remote_del = " << p_remote_del
       << endl;

	m_PO_PC_Logger->Write(ss.str());

    E_local_toSend = m_E_local_IN;

	return control_signal;
}

void PO_PC_Slave_YoubotArm::Passivity_Observer(double f, double v, double PO_DZ_Width)
{
	double P_local = f * v;
    P_local = DeadZone(P_local, PO_DZ_Width);

	if ( P_local < 0)
	{
        m_E_local_IN -= m_T * P_local;
	}
	else // P_local > 0
	{
        m_E_local_OUT += m_T * P_local;
	}

    m_PC_Energy -= m_T * m_PC_signal_old * m_f_old;
}

double PO_PC_Slave_YoubotArm::Passivity_Controller(double f_local, double v_local, double f_remote_del, double v_remote_del, double E_IN_remote_del)
{    
    m_PC_signal = 0;
    
    if (m_PO_PC_Type == No_PO_PC)
    {
        m_ControlSignal = f_remote_del;
    }
    else if (m_PO_PC_Type == Original_PO_PC)
    {
        if(((m_E_local_OUT + m_PC_Energy - E_IN_remote_del) > 0) && (abs(f_local) >= m_f_S_treshold)) //!!!! >0 artificial delay  ; <0 no artifical delay
        {
            m_PC_signal_F = (m_E_local_OUT + m_PC_Energy - E_IN_remote_del)  / (m_T * f_local);
        }

        if (m_PC_signal_F > m_max_velocity)
        {
            m_PC_signal_F = m_max_velocity;
        }
        if (m_PC_signal_F < -m_max_velocity)
        {
            m_PC_signal_F = -m_max_velocity;
        }

        m_ControlSignal = v_remote_del - m_PC_signal_F;

        if (v_remote_del == 0)
        {
          m_ControlSignal = 0;
        }

        if (v_remote_del > 0 && m_ControlSignal < 0)
        {
          m_ControlSignal = 0;
        }

        if (v_remote_del < 0 && m_ControlSignal > 0)
        {
          m_ControlSignal = 0;
        }
    }
    else if (m_PO_PC_Type == Delta_PO_PC)
    {
        if ((m_E_local_OUT + m_PC_Energy - E_IN_remote_del) > 0 )
        {
            double delta_fun = 1.0 / (f_local + Sign(f_local) * m_delta_S);
            m_PC_signal_F = delta_fun * ((m_E_local_OUT + m_PC_Energy - E_IN_remote_del) / m_T  + m_z);
            m_z += (1.0/m_TI_S) * ( (m_E_local_OUT + m_PC_Energy - E_IN_remote_del) / m_T  - (m_PC_signal * v_local) );
        }
        else
        {
            m_z = 0.0;
        }

        if (m_PC_signal_F > m_max_velocity)
        {
            m_PC_signal_F = m_max_velocity;
        }
        if (m_PC_signal_F < -m_max_velocity)
        {
            m_PC_signal_F = -m_max_velocity;
        }

        m_ControlSignal = v_remote_del - m_PC_signal_F;

        if (v_remote_del == 0)
        {
          m_ControlSignal = 0;
        }

        if (v_remote_del > 0 && m_ControlSignal < 0)
        {
          m_ControlSignal = 0;
        }

        if (v_remote_del < 0 && m_ControlSignal > 0)
        {
          m_ControlSignal = 0;
        }
    }

    return m_ControlSignal;
}

