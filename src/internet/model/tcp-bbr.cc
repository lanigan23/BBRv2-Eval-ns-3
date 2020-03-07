/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 NITK Surathkal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Vivek Jain <jain.vivek.anand@gmail.com>
 *          Viyom Mittal <viyommittal@gmail.com>
 *          Mohit P. Tahiliani <tahiliani@nitk.edu.in>
 */

#include "tcp-bbr.h"
#include "ns3/log.h"
#include "ns3/tcp-socket-base.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpBbr");
NS_OBJECT_ENSURE_REGISTERED (TcpBbr);

const double TcpBbr::PACING_GAIN_CYCLE [] = {5.0 / 4, 3.0 / 4, 1, 1, 1, 1, 1, 1};
const double TcpBbr::PACING_GAIN_CYCLE_HSR [] = {3.0 / 2, 0.5, 3.0 /2, 0.5, 3.0 / 2, 0.5, 3.0 / 2, 0.5};
const double TcpBbr::PACING_GAIN_CYCLE_DELAY [] = {1.11, 0.9, 1, 1, 1, 1, 1, 1};

TypeId
TcpBbr::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpBbr")
    .SetParent<TcpCongestionOps> ()
    .AddConstructor<TcpBbr> ()
    .SetGroupName ("Internet")
    .AddAttribute ("HighGain",
                   "Value of high gain",
                   DoubleValue (2.89),
                   MakeDoubleAccessor (&TcpBbr::m_highGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BwWindowLength",
                   "Length of bandwidth windowed filter",
                   UintegerValue (10),
                   MakeUintegerAccessor (&TcpBbr::m_bandwidthWindowLength),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("RttWindowLength",
                   "Length of bandwidth windowed filter",
                   TimeValue (Seconds (10)),
                   MakeTimeAccessor (&TcpBbr::m_rtPropFilterLen),
                   MakeTimeChecker ())
    .AddAttribute ("ProbeRttDuration",
                   "Length of bandwidth windowed filter",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&TcpBbr::m_probeRttDuration),
                   MakeTimeChecker ())
    .AddAttribute ("BBRVariant",
                   "Variant of BBR to use",
                   EnumValue (TcpBbr::BBR),
                   MakeEnumAccessor (&TcpBbr::m_variant),
                   MakeEnumChecker (BbrVar::BBR, "BBR",
                                    BbrVar::BBR_PRIME, "BBR Prime",
                                    BbrVar::BBR_PLUS, "BBR Plus",
                                    BbrVar::BBR_HSR, "BBR+",
                                    BbrVar::BBR_DELAY, "Delay-BBR",
                                    BbrVar::BBR_V2, "BBR V2"))
    .AddAttribute ("RTPropLambda",
                   "Value of lambda to use for BBR+ RtProp estimation",
                   UintegerValue (1/8),
                   MakeUintegerAccessor (&TcpBbr::m_lambda),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("EnableEcn",
                   "Whether to use Ecn or not",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TcpBbr::m_enableEcn),
                   MakeBooleanChecker ())
  ;
  return tid;
}

TcpBbr::TcpBbr ()
  : TcpCongestionOps ()
{
  NS_LOG_FUNCTION (this);
  m_uv = CreateObject<UniformRandomVariable> ();
}

TcpBbr::TcpBbr (const TcpBbr &sock)
  : TcpCongestionOps (sock),
    m_bandwidthWindowLength (sock.m_bandwidthWindowLength),
    m_pacingGain (sock.m_pacingGain),
    m_cWndGain (sock.m_cWndGain),
    m_highGain (sock.m_highGain),
    m_isPipeFilled (sock.m_isPipeFilled),
    m_minPipeCwnd (sock.m_minPipeCwnd),
    m_roundCount (sock.m_roundCount),
    m_roundStart (sock.m_roundStart),
    m_nextRoundDelivered (sock.m_nextRoundDelivered),
    m_probeRttDuration (sock.m_probeRttDuration),
    m_probeRtPropStamp (sock.m_probeRtPropStamp),
    m_probeRttDoneStamp (sock.m_probeRttDoneStamp),
    m_probeRttRoundDone (sock.m_probeRttRoundDone),
    m_packetConservation (sock.m_packetConservation),
    m_priorCwnd (sock.m_priorCwnd),
    m_idleRestart (sock.m_idleRestart),
    m_targetCWnd (sock.m_targetCWnd),
    m_fullBandwidth (sock.m_fullBandwidth),
    m_fullBandwidthCount (sock.m_fullBandwidthCount),
    m_rtProp (Time::Max ()),
    m_sendQuantum (sock.m_sendQuantum),
    m_cycleStamp (sock.m_cycleStamp),
    m_cycleIndex (sock.m_cycleIndex),
    m_rtPropExpired (sock.m_rtPropExpired),
    m_rtPropFilterLen (sock.m_rtPropFilterLen),
    m_rtPropStamp (sock.m_rtPropStamp),
    m_isInitialized (sock.m_isInitialized)
{
  NS_LOG_FUNCTION (this);
  m_uv = CreateObject<UniformRandomVariable> ();
}

int64_t
TcpBbr::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uv->SetStream (stream);
  return 1;
}

void
TcpBbr::InitRoundCounting ()
{
  NS_LOG_FUNCTION (this);
  m_nextRoundDelivered = 0;
  m_roundStart = false;
  m_roundCount = 0;
}

void
TcpBbr::InitFullPipe ()
{
  NS_LOG_FUNCTION (this);
  m_isPipeFilled = false;
  m_fullBandwidth = 0;
  m_fullBandwidthCount = 0;
}

void
TcpBbr::InitPacingRate (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  if (!tcb->m_pacing)
    {
      NS_LOG_WARN ("BBR must use pacing");
      tcb->m_pacing = true;
    }
  Time rtt = tcb->m_lastRtt != Time::Max () ? tcb->m_lastRtt : MilliSeconds (1);
  DataRate nominalBandwidth (tcb->m_initialCWnd * tcb->m_segmentSize * 8 / rtt.GetSeconds ());
  tcb->m_currentPacingRate = DataRate (m_pacingGain * nominalBandwidth.GetBitRate ());
}

void
TcpBbr::EnterStartup ()
{
  NS_LOG_FUNCTION (this);
  SetBbrState (BbrMode_t::BBR_STARTUP);
  m_pacingGain = m_highGain;
  m_cWndGain = m_highGain;
}

void
TcpBbr::HandleRestartFromIdle (Ptr<TcpSocketState> tcb, const RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (tcb->m_bytesInFlight == 0 && rs->m_isAppLimited)
    {
      m_idleRestart = true;
      if (m_state == BbrMode_t::BBR_PROBE_BW)
        {
          SetPacingRate (tcb, 1);
        }
    }
}

void
TcpBbr::SetPacingRate (Ptr<TcpSocketState> tcb, double gain)
{
  NS_LOG_FUNCTION (this << tcb << gain);
  DataRate rate (gain * m_maxBwFilter.GetBest ().GetBitRate ());
  rate = std::min (rate, tcb->m_maxPacingRate);
  if (m_isPipeFilled || rate > tcb->m_currentPacingRate)
    {
      tcb->m_currentPacingRate = rate;
    }
}

uint32_t
TcpBbr::InFlight (Ptr<TcpSocketState> tcb, double gain)
{
  NS_LOG_FUNCTION (this << tcb << gain);
  if (m_rtProp == Time::Max ())
    {
      return tcb->m_initialCWnd * tcb->m_segmentSize;
    }
  double quanta = 3 * m_sendQuantum;
  double estimatedBdp = m_maxBwFilter.GetBest () * m_rtProp / 8.0;
  return gain * estimatedBdp + quanta;
}

void
TcpBbr::AdvanceCyclePhase ()
{
  NS_LOG_FUNCTION (this);
  m_cycleStamp = Simulator::Now ();
  m_cycleIndex = (m_cycleIndex + 1) % GAIN_CYCLE_LENGTH;
  if (m_variant == BbrVar::BBR_HSR) 
    {
      m_pacingGain = PACING_GAIN_CYCLE_HSR [m_cycleIndex];
    }
  else if (m_variant == TcpBbr::BBR_DELAY)
    {
      m_pacingGain = PACING_GAIN_CYCLE_DELAY [m_cycleIndex];
    }
  else
    {
      m_pacingGain = PACING_GAIN_CYCLE [m_cycleIndex];
    }  
}

bool
TcpBbr::IsNextCyclePhase (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  bool isFullLength = (Simulator::Now () - m_cycleStamp) > m_rtProp;
  if (m_pacingGain == 1)
    {
      return isFullLength;
    }
  else if (m_pacingGain > 1)
    {
      return isFullLength && (rs->m_packetLoss > 0 || tcb->m_priorInFlight >= InFlight (tcb, m_pacingGain));
    }
  else
    {
      if (m_variant == BbrVar::BBR)
        {
          return isFullLength;
        }
      else if (m_variant == BbrVar::BBR_PRIME)
        {              
          return tcb->m_priorInFlight <= InFlight (tcb, 1);
        }
      else
        {
          return isFullLength || tcb->m_priorInFlight <= InFlight (tcb, 1);
        } 
    }
}

void
TcpBbr::CheckCyclePhase (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (m_state == BbrMode_t::BBR_PROBE_BW && m_variant == BbrVar::BBR_PLUS)
    {
      DrainToTargetCycling (tcb, rs);
    }
  else if (m_state == BbrMode_t::BBR_PROBE_BW && IsNextCyclePhase (tcb, rs))
    {
      AdvanceCyclePhase ();
    }
}

void
TcpBbr::CheckFullPipe (const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << rs);
  if (m_isPipeFilled || !m_roundStart || rs->m_isAppLimited)
    {
      return;
    }

  /* Check if Bottleneck bandwidth is still growing*/
  if (m_maxBwFilter.GetBest ().GetBitRate () >= m_fullBandwidth.GetBitRate () * 1.25)
    {
      m_fullBandwidth = m_maxBwFilter.GetBest ();
      m_fullBandwidthCount = 0;
      return;
    }

  m_fullBandwidthCount++;
  if (m_fullBandwidthCount >= 3)
    {
      NS_LOG_DEBUG ("Pipe filled");
      m_isPipeFilled = true;
    }
}

void
TcpBbr::EnterDrain ()
{
  NS_LOG_FUNCTION (this);
  SetBbrState (BbrMode_t::BBR_DRAIN);
  m_pacingGain = 1.0 / m_highGain;
  m_cWndGain = m_highGain;
}

void
TcpBbr::EnterProbeBW (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this);
  SetBbrState (BbrMode_t::BBR_PROBE_BW);
  if (m_variant == BbrVar::BBR_V2 && m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_NS)
    {
      EnterProbeDown (tcb);
    }
  else if (m_variant == BbrVar::BBR_V2 && m_cycleIndex != BbrBwPhase::BBR_BW_PROBE_NS)
    {
      EnterProbeDown (tcb);
      EnterProbeCruise ();
    }
  else
    {
      m_pacingGain = 1;
      m_cWndGain = 2;
      m_cycleIndex = GAIN_CYCLE_LENGTH - 1 - (int) m_uv->GetValue (0, 8);
      AdvanceCyclePhase ();
    }
}

void
TcpBbr::CheckDrain (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  if (m_state == BbrMode_t::BBR_STARTUP && m_isPipeFilled)
    {
      EnterDrain ();
      ResetCongestionSignals ();
    }

  if (m_state == BbrMode_t::BBR_DRAIN && tcb->m_bytesInFlight <= InFlight (tcb, 1))
    {
      m_cycleIndex = BbrBwPhase::BBR_BW_PROBE_NS;
      EnterProbeBW (tcb);
      m_sentPacketNum = tcb->m_lastSentSeq;
    }
}

void
TcpBbr::UpdateRTprop (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  m_rtPropExpired = Simulator::Now () > (m_rtPropStamp + m_rtPropFilterLen);
  if (tcb->m_lastRtt >= Seconds (0) && (tcb->m_lastRtt <= m_rtProp || m_rtPropExpired))
    {
      if (m_variant == BbrVar::BBR_HSR)
        {
          double rttVar = sqrt((tcb->m_rttVar).GetDouble ());
          m_rtProp = tcb->m_lastRtt + m_lambda * MilliSeconds (rttVar);
        }
      else 
        {
          m_rtProp = tcb->m_lastRtt;
        }
      m_rtPropStamp = Simulator::Now ();
    }
}

void
TcpBbr::EnterProbeRTT ()
{
  NS_LOG_FUNCTION (this);
  SetBbrState (BbrMode_t::BBR_PROBE_RTT);
  if (m_congestionDelay)
    {
      m_pacingGain = 0.75;
      m_cWndGain = 0.75;
    }
  else 
    {
      m_pacingGain = 1;
      m_cWndGain = 1;
    }
}

void
TcpBbr::SaveCwnd (Ptr<const TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  if (tcb->m_congState != TcpSocketState::CA_RECOVERY && m_state != BbrMode_t::BBR_PROBE_RTT)
    {
      m_priorCwnd = tcb->m_cWnd;
    }
  else
    {
      m_priorCwnd = std::max (m_priorCwnd, tcb->m_cWnd.Get ());
    }
}

void
TcpBbr::RestoreCwnd (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  tcb->m_cWnd = std::max (m_priorCwnd, tcb->m_cWnd.Get ());
}

void
TcpBbr::ExitProbeRTT (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this);
  // Reset lower bounds
  m_inflightLo = std::numeric_limits<int>::max ();
  m_bwLo = std::numeric_limits<int>::max ();
  if (m_isPipeFilled)
    {
      EnterProbeBW (tcb);
    }
  else
    {
      EnterStartup ();
    }
}

void
TcpBbr::HandleProbeRTT (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  tcb->m_appLimited = (tcb->m_delivered + tcb->m_bytesInFlight) ? : 1;

  if (m_probeRttDoneStamp == Seconds (0) && 
  ((tcb->m_bytesInFlight <= m_minPipeCwnd && m_variant != BbrVar::BBR_DELAY) 
  || (tcb->m_bytesInFlight <= InFlight (tcb, 0.75) && m_variant == BbrVar::BBR_DELAY)))
    {
      m_probeRttDoneStamp = Simulator::Now () + m_probeRttDuration;
      m_probeRttRoundDone = false;
      m_nextRoundDelivered = tcb->m_delivered;
    }
  else if (m_probeRttDoneStamp != Seconds (0))
    {
      if (m_roundStart)
        {
          m_probeRttRoundDone = true;
        }
      if (m_probeRttRoundDone && Simulator::Now () > m_probeRttDoneStamp)
        {
          m_rtPropStamp = Simulator::Now ();
          m_baseRtt = Time::Max ();
          m_srtt = Seconds (0);
          RestoreCwnd (tcb);
          ExitProbeRTT (tcb);
        }
    }
}

void
TcpBbr::CheckProbeRTT (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  NS_LOG_DEBUG (Simulator::Now () << "WhichState " << WhichState (m_state) << " m_rtPropExpired " << m_rtPropExpired << " !m_idleRestart " << !m_idleRestart);
  if (m_state != BbrMode_t::BBR_PROBE_RTT && (m_rtPropExpired || m_congestionDelay) && !m_idleRestart)
    {
      EnterProbeRTT ();
      SaveCwnd (tcb);
      m_probeRttDoneStamp = Seconds (0);
      m_ackPhase = BbrAckPhase::BBR_ACK_PROBE_STOPPING;
    }

  if (m_state == BbrMode_t::BBR_PROBE_RTT)
    {
      HandleProbeRTT (tcb);
    }

  m_idleRestart = false;
}

void
TcpBbr::SetSendQuantum (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  m_sendQuantum = 1 * tcb->m_segmentSize;
/*TODO
  Since TSO can't be implemented in ns-3
  if (tcb->m_currentPacingRate < DataRate ("1.2Mbps"))
    {
      m_sendQuantum = 1 * tcb->m_segmentSize;
    }
  else if (tcb->m_currentPacingRate < DataRate ("24Mbps"))
    {
      m_sendQuantum  = 2 * tcb->m_segmentSize;
    }
  else
    {
      m_sendQuantum = std::min (tcb->m_currentPacingRate.GetBitRate () * MilliSeconds (1).GetMilliSeconds () / 8, (uint64_t) 64000);
    }*/
}

void
TcpBbr::UpdateTargetCwnd (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  m_targetCWnd = InFlight (tcb, m_cWndGain);
}

void
TcpBbr::ModulateCwndForRecovery (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if ( rs->m_packetLoss > 0)
    {
      tcb->m_cWnd = std::max (tcb->m_cWnd.Get () - rs->m_packetLoss, tcb->m_segmentSize);
    }

  if (m_packetConservation)
    {
      tcb->m_cWnd = std::max (tcb->m_cWnd.Get (), tcb->m_bytesInFlight + tcb->m_lastAckedSackedBytes);
    }
}

void
TcpBbr::ModulateCwndForProbeRTT (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  if (m_state == BbrMode_t::BBR_PROBE_RTT)
    {
      if (m_variant == BbrVar::BBR_DELAY)
        {
          tcb->m_cWnd = InFlight (tcb, 0.75);
        }
      else
        {
          tcb->m_cWnd = std::min (tcb->m_cWnd.Get (), m_minPipeCwnd);
        }
    }
}

void
TcpBbr::SetCwnd (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  UpdateTargetCwnd (tcb);

  if (tcb->m_congState == TcpSocketState::CA_RECOVERY)
    {
      ModulateCwndForRecovery (tcb, rs);
    }

  if (!m_packetConservation)
    {
      if (m_isPipeFilled)
        {
          tcb->m_cWnd = std::min (tcb->m_cWnd.Get () + (uint32_t) tcb->m_lastAckedSackedBytes, m_targetCWnd);
        }
      else if (tcb->m_cWnd < m_targetCWnd || tcb->m_delivered < tcb->m_initialCWnd * tcb->m_segmentSize)
        {
          tcb->m_cWnd = tcb->m_cWnd.Get () + tcb->m_lastAckedSackedBytes;
        }
      tcb->m_cWnd = std::max (tcb->m_cWnd.Get (), m_minPipeCwnd);
    }
  ModulateCwndForProbeRTT (tcb);
  if (tcb->m_congState == TcpSocketState::CA_RECOVERY)
    {
      m_packetConservation = false;
    }
}

void
TcpBbr::UpdateRound (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (tcb->m_txItemDelivered >= m_nextRoundDelivered)
    {
      m_nextRoundDelivered = tcb->m_delivered;
      m_roundCount++;
      m_roundStart = true;
    }
  else
    {
      m_roundStart = false;
    }
}

void
TcpBbr::UpdateBtlBw (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (rs->m_deliveryRate == 0)
    {
      return;
    }

  UpdateRound (tcb, rs);

  if (rs->m_deliveryRate >= m_maxBwFilter.GetBest () || !rs->m_isAppLimited)
    {
      m_maxBwFilter.Update (rs->m_deliveryRate, m_roundCount);
    }
}

void
TcpBbr::UpdateModelAndState (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  UpdateBtlBw (tcb, rs);
  if (m_variant == BbrVar::BBR_DELAY)
    {
      CheckCongestionDelay (tcb);
    }
  if (m_variant != BbrVar::BBR_V2)
    {
      CheckCyclePhase (tcb, rs);
    }
  if (m_variant == BbrVar::BBR_V2)
    {
      if (m_roundStart)
        {
          m_roundsSinceProbe = m_roundsSinceProbe + 1;
          UpdateEcn (tcb, rs);
        }
      UpdateCongestionSignals (tcb, rs);
      CheckExcessiveLossStartup (tcb, rs);
    }
  CheckFullPipe (rs);
  CheckDrain (tcb);
  if (m_variant == BbrVar::BBR_V2)
    {
      UpdateCyclePhase (tcb, rs);
    }
  UpdateRTprop (tcb);
  CheckProbeRTT (tcb);
}

void
TcpBbr::UpdateControlParameters (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  SetPacingRate (tcb, m_pacingGain);
  SetSendQuantum (tcb);
  SetCwnd (tcb, rs);
  if (m_variant == BbrVar::BBR_V2)
    {
      BoundCwndForInflightModel (tcb);
    }
}

std::string
TcpBbr::WhichState (BbrMode_t mode) const
{
  switch (mode)
    {
    case 0:
      return "BBR_STARTUP";
    case 1:
      return "BBR_DRAIN";
    case 2:
      return "BBR_PROBE_BW";
    case 3:
      return "BBR_PROBE_RTT";
    }
  NS_ASSERT (false);
}

void
TcpBbr::SetBbrState (BbrMode_t mode)
{
  NS_LOG_FUNCTION (this << mode);
  NS_LOG_DEBUG (Simulator::Now () << " Changing from " << WhichState (m_state) << " to " << WhichState (mode));
  m_state = mode;
}

uint32_t
TcpBbr::GetBbrState ()
{
  NS_LOG_FUNCTION (this);
  return m_state;
}

double
TcpBbr::GetCwndGain ()
{
  NS_LOG_FUNCTION (this);
  return m_cWndGain;
}

double
TcpBbr::GetPacingGain ()
{
  NS_LOG_FUNCTION (this);
  return m_pacingGain;
}

std::string
TcpBbr::GetName () const
{
  return "TcpBbr";
}

bool
TcpBbr::HasCongControl () const
{
  NS_LOG_FUNCTION (this);
  return true;
}

void
TcpBbr::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                        const Time& rtt)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);
  CongControl (tcb, &tcb->m_rs);
}

void
TcpBbr::CongControl (Ptr<TcpSocketState> tcb, const struct RateSample *rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  UpdateModelAndState (tcb, rs);
  UpdateControlParameters (tcb, rs);
}

void
TcpBbr::CongestionStateSet (Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this << tcb << newState);
  if (newState == TcpSocketState::CA_OPEN && !m_isInitialized)
    {
      NS_LOG_DEBUG ("CongestionStateSet triggered to CA_OPEN :: " << newState);
      m_rtProp = tcb->m_lastRtt != Time::Max () ? tcb->m_lastRtt : Time::Max ();
      m_rtPropStamp = Simulator::Now ();
      m_priorCwnd = tcb->m_initialCWnd * tcb->m_segmentSize;
      m_targetCWnd = tcb->m_initialCWnd * tcb->m_segmentSize;
      m_minPipeCwnd = 4 * tcb->m_segmentSize;
      m_sendQuantum = 1 * tcb->m_segmentSize;
      m_maxBwFilter = MaxBandwidthFilter_t (m_bandwidthWindowLength, DataRate (0), 0);
      InitRoundCounting ();
      InitFullPipe ();
      EnterStartup ();
      InitPacingRate (tcb);
      m_isInitialized = true;
    }
  else if (newState == TcpSocketState::CA_LOSS)
    {
      NS_LOG_DEBUG ("CongestionStateSet triggered to CA_LOSS :: " << newState);
      SaveCwnd (tcb);
      m_roundStart = true;
      if (IsProbingBandwidth () && m_inflightLo == std::numeric_limits<int>::max ())
        {
          m_inflightLo = m_priorCwnd;
        }
    }
  else if (newState == TcpSocketState::CA_RECOVERY)
    {
      NS_LOG_DEBUG ("CongestionStateSet triggered to CA_RECOVERY :: " << newState);
      SaveCwnd (tcb);
      tcb->m_cWnd = tcb->m_bytesInFlight + std::max (tcb->m_lastAckedSackedBytes, tcb->m_segmentSize);
      m_packetConservation = true;
    }
}

void
TcpBbr::CwndEvent (Ptr<TcpSocketState> tcb,
                   const TcpSocketState::TcpCAEvent_t event)
{
  NS_LOG_FUNCTION (this << tcb << event);
  if (event == TcpSocketState::CA_EVENT_COMPLETE_CWR)
    {
      NS_LOG_DEBUG ("CwndEvent triggered to CA_EVENT_COMPLETE_CWR :: " << event);
      m_packetConservation = false;
      RestoreCwnd (tcb);
    }
  else if (event == TcpSocketState::CA_EVENT_TX_START)
    {
      NS_LOG_DEBUG ("CwndEvent triggered to CA_EVENT_TX_START :: " << event);
      if (tcb->m_bytesInFlight == 0 && tcb->m_appLimited)
        {
          m_idleRestart = true;
          if (m_state == BbrMode_t::BBR_PROBE_BW && tcb->m_appLimited)
            {
              SetPacingRate (tcb, 1);
            }
        }
    }
  else if (event == TcpSocketState::CA_EVENT_ECN_IS_CE)
    {
      m_deliveredEce += tcb->m_delivered;
      if (tcb->m_txItemDelivered > tcb->m_rs.m_priorDelivered)
        {
          tcb->m_rs.m_priorDeliveredEce = tcb->m_txItemDelivered;
        }
      tcb->m_rs.m_deliveredEce = tcb->m_delivered - tcb->m_rs.m_priorDeliveredEce;
    }
}

uint32_t
TcpBbr::GetBbrVariant ()
{
  NS_LOG_FUNCTION (this);
  return m_variant;
}

void 
TcpBbr::SetBbrVariant (BbrVar variant)
{
  NS_LOG_FUNCTION (this << variant);
  m_variant = variant;
}

void
TcpBbr::DrainToTargetCycling (Ptr<TcpSocketState> tcb, const struct RateSample *rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (m_state != BbrMode_t::BBR_PROBE_BW)
    {
      return;
    }
  
  if ((Simulator::Now () - m_cycleStamp) > m_cycleLength * m_rtProp)
    {
      m_cycleStamp = Simulator::Now ();
      m_cycleLength = GAIN_CYCLE_LENGTH - (int) m_uv->GetValue (0, m_cycleRand);
      SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_UP);
      return;
    }

  if (m_pacingGain == PACING_GAIN_CYCLE [BbrBwPhase::BBR_BW_PROBE_CRUISE])
    {
      return;
    }

  if (m_pacingGain < PACING_GAIN_CYCLE [BbrBwPhase::BBR_BW_PROBE_CRUISE])
    {
      if (tcb->m_priorInFlight <= InFlight (tcb, 1))
        {
          SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_CRUISE);
        }
      return;
    }
  if ((Simulator::Now () - m_cycleStamp) > m_rtProp && ((tcb->m_priorInFlight <= InFlight (tcb, m_pacingGain)) || 
        rs->m_packetLoss > 0 || rs->m_isAppLimited))
    {
      SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_DOWN);
      return;
    }
}

void
TcpBbr::SetCycleIndex (BbrBwPhase index)
{
  NS_LOG_FUNCTION (this << index);
  m_cycleIndex = index;
  m_pacingGain = PACING_GAIN_CYCLE [m_cycleIndex];
}

void
TcpBbr::CheckCongestionDelay (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  if (m_state == BbrMode_t::BBR_PROBE_BW && tcb->m_lastAckedSeq > m_sentPacketNum)
    {
      if (m_baseRtt != Time::Max ())
        {
          int64_t m_old_srtt = m_srtt.GetMilliSeconds ();
          int64_t m_new_rtt = tcb->m_lastRtt.GetMilliSeconds ();
          int64_t m_new_srtt = (1 - m_alphaSrtt) * m_old_srtt + m_alphaSrtt * m_new_rtt;
          m_srtt = MilliSeconds (m_new_srtt);

          if (m_srtt < m_baseRtt)
            {
              m_baseRtt = m_srtt; 
            }
        }
      else 
        {
          m_srtt = tcb->m_lastRtt;
          m_baseRtt = m_srtt;
        }
    }
  if (m_baseRtt != Time::Max () && m_state == BbrMode_t::BBR_PROBE_BW && m_srtt > (m_baseRtt * m_beta))
    {
      m_congestionDelay = true;
      m_sentPacketNum = tcb->m_lastSentSeq;
    }
}

void
TcpBbr::UpdateCongestionSignals (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  m_lossRoundStart = 0;
  if (rs->m_interval <= 0 || !rs->m_lastAckedSackedBytes)
    {
      return;
    }

  if (rs->m_deliveryRate > m_maxBwFilter.GetBest () || !rs->m_isAppLimited)
    {
      m_maxBwFilter.Update (rs->m_deliveryRate, m_roundCount);
      m_bwHi[1] = m_maxBwFilter.GetBest ();
    }

  m_lossInRound |= (rs->m_packetLoss > 0);

  // Update bw and inflight data from latest round trip
  m_bwLatest = m_bwLatest > rs->m_deliveryRate ? m_bwLatest : rs->m_deliveryRate;
  m_inflightLatest = m_inflightLatest > rs->m_delivered ? m_inflightLatest : rs->m_delivered;

  if ((rs->m_priorDelivered - m_lossRoundDelivered) > 0)
    {
      return;
    }
  
  // Per round trip updates
  m_lossRoundDelivered = tcb->m_delivered;
  m_lossRoundStart = 1;
  AdaptLowerBounds (tcb);

  // Update latest filters
  m_lossInRound = 0;
  m_ecnInRound = 0;
  m_bwLatest = rs->m_deliveryRate;
  m_inflightLatest = rs->m_delivered;
}

void
TcpBbr::ResetCongestionSignals ()
{
  NS_LOG_FUNCTION (this);
  m_lossInRound = 0;
  m_ecnInRound = 0;
  m_bwLatest = 0;
  m_inflightLatest = 0;
}

bool
TcpBbr::IsProbingBandwidth ()
{
  NS_LOG_FUNCTION (this);
  return m_state == BbrMode_t::BBR_STARTUP || (m_state == BbrMode_t::BBR_PROBE_BW && 
            (m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_UP || m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_REFILL));
}

void
TcpBbr::AdaptLowerBounds (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  // Don't adjust lower bounds if probing for bandwidth
  if (IsProbingBandwidth ())
    {
      return;
    }

  uint32_t ecnInflightLo = std::numeric_limits<int>::max ();

  // Adjust to ECN
  if (m_ecnInRound && m_enableEcn)
    {
      if (m_inflightLo == std::numeric_limits<int>::max ())
        {
          m_inflightLo = tcb->m_cWnd;
        }
      ecnInflightLo = m_inflightLo * (1 - (m_ecnAlpha * m_ecnFactor));
    }
  
  // Adjust to loss
  if (m_lossInRound)
    {
      if (m_bwLo == std::numeric_limits<int>::max ())
        {
          m_bwLo = m_bwHi[0] > m_bwHi[1] ? m_bwHi[0] : m_bwHi[1];
        }
      if (m_inflightLo == std::numeric_limits<int>::max ())
        {
          m_inflightLo = tcb->m_cWnd;
        }
      uint64_t m_bwLoTmp = m_bwLo.GetBitRate () * (1 - m_bbrBeta);
      DataRate m_bwLoRes (m_bwLoTmp);
      m_bwLo = std::max (m_bwLatest, m_bwLoRes);
      m_inflightLo = std::max (m_inflightLatest, m_inflightLo * (1 - m_bbrBeta));
    }

    m_inflightLo = std::min (m_inflightLo, ecnInflightLo);
}

bool
TcpBbr::AdaptUpperBounds (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  
  // Track bw/loss samples when probing
  if (m_ackPhase == BbrAckPhase::BBR_ACK_PROBE_STARTING && m_roundStart)
    {
      m_ackPhase = BbrAckPhase::BBR_ACK_PROBE_FEEDBACK;
    }

  // End sampling from probing phase 
  if (m_ackPhase == BbrAckPhase::BBR_ACK_PROBE_STOPPING && m_roundStart)
    {
      m_bwProbeSamples = 0;
      m_ackPhase = BbrAckPhase::BBR_ACKS_INIT;
      if (m_state == BbrMode_t::BBR_PROBE_BW && !rs->m_isAppLimited)
        {
          // Current bw sample is best recent max bw so advance window to forget previous samples
          AdvanceBwHiFilter ();
        }

      if (m_state == BbrMode_t::BBR_PROBE_BW && m_isRiskyProbe && !m_prevProbeTooHigh)
        {
          // If probe hit inflight_hi without loss, probe again
          EnterProbeRefill (tcb, 0);
          return true;
        }
    }

  if (IsInflightTooHigh (tcb, rs))
    {
      if (m_bwProbeSamples > 0)
        {
          // Handle if sample is from bw probing
          HandleInflightTooHigh (tcb, rs);
        }
    }
  else
    {
      // Safe loss rate, adjust upper bound
      if (m_inflightHi == std::numeric_limits<int>::max ())
        {
          return false;
        }
      
      if (tcb->m_bytesInFlight > m_inflightHi)
        {
          m_inflightHi = tcb->m_bytesInFlight;
        }
      
      if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_UP)
        {
          ProbeInflightHighUpward (tcb, rs);
        }
    }

  return false;
}

void
TcpBbr::CheckExcessiveLossStartup (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);

  if (m_state != BbrMode_t::BBR_STARTUP)
    {
      return;
    }

  if (m_isPipeFilled)
    {
      return;
    }

  // Check loss rate at end of each round trip
  if (rs->m_packetLoss && m_startupLossEvents < 0xf)
    {
      m_startupLossEvents++;
    }

  // Exit if loss is too high
  if (m_fullLossCount && m_lossRoundStart && tcb->m_congState == TcpSocketState::CA_RECOVERY &&
        m_startupLossEvents >= m_fullLossCount && IsInflightTooHigh (tcb, rs))
    {
      m_isPipeFilled = true;
      m_inflightHi = InFlight(tcb, 1);
    }

  if (m_lossRoundStart)
    {
      m_startupLossEvents = 0;
    }
}

bool
TcpBbr::IsInflightTooHigh (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);

  if (rs->m_packetLoss > 0 && tcb->m_bytesInFlight > 0)
    {
      if (rs->m_packetLoss > (tcb->m_bytesInFlight * m_lossThresh))
        {
          return true;
        }
    }

  if (rs->m_deliveredEce > 0 && rs->m_delivered > 0 && m_enableEcn)
    {
      if (rs->m_deliveredEce >= rs->m_delivered * m_ecnThresh)
        {
          return true;
        }
    }
  return false;
}

void
TcpBbr::HandleInflightTooHigh (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  m_prevProbeTooHigh = true;
  m_bwProbeSamples = 0;
  if (!rs->m_isAppLimited)
    {
      m_inflightHi = std::max (tcb->m_bytesInFlight, TargetInflight () * (1 - m_bbrBeta));
    }
  if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_UP)
    {
      EnterProbeDown (tcb);
    }
}

void
TcpBbr::AdvanceBwHiFilter ()
{
  NS_LOG_FUNCTION (this);

  if (m_bwHi[1] == 0)
    {
      return;
    }
  m_bwHi[0] = m_bwHi[1];
  m_bwHi[1] = 0;
}

uint32_t
TcpBbr::TargetInflight ()
{
  NS_LOG_FUNCTION (this);
  DataRate maxBw = m_bwHi[0] > m_bwHi[1] ? m_bwHi[0] : m_bwHi[1];
  DataRate estBw = maxBw > m_bwLo ? m_bwLo : maxBw;
  double bdp = estBw * m_rtProp / 8.0;
  double quanta = 3 * m_sendQuantum;
  return bdp + quanta;
}

void
TcpBbr::ProbeInflightHighUpward (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);

  // Don't grow inflight_hi if it is not fully used
  if (!(tcb->m_priorInFlight >= m_priorCwnd) || tcb->m_initialCWnd < m_inflightHi)
    {
      m_bwProbeUpAcks = 0;
      return;
    }

  // For each probe up count packet Acked, increase inflight_hi by 1
  m_bwProbeUpAcks += rs->m_lastAckedSackedBytes;

  if (m_bwProbeUpAcks >= m_bwProbeUpCount)
    {
      m_bwProbeUpAcks -= (m_bwProbeUpAcks / m_bwProbeUpCount) * m_bwProbeUpCount;
      m_inflightHi += m_bwProbeUpAcks / m_bwProbeUpCount;
    }

  if (m_roundStart)
    {
      RaiseInflightHiSlope (tcb);
    }
}

void
TcpBbr::RaiseInflightHiSlope (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  // Calculate packets (S)Acked per inflight_hi increment
  uint32_t roundGrowth = 1 << m_bwProbeUpRounds;
  m_bwProbeUpRounds = std::min<uint32_t> (m_bwProbeUpRounds + 1, 30);
  m_bwProbeUpCount = std::max<uint32_t> (tcb->m_initialCWnd / roundGrowth, 1);
}

uint32_t
TcpBbr::InflightWithHeadroom ()
{
  NS_LOG_FUNCTION (this);

  if (m_inflightHi == std::numeric_limits<int>::max ())
    {
      return std::numeric_limits<int>::max ();
    }

  uint32_t headroom = m_inflightHi * m_inflightHeadroom;
  return std::max (m_inflightHi - headroom, m_minPipeCwnd);
}

bool
TcpBbr::IsTimeToProbe (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  bool isFullLength = (Simulator::Now () - m_cycleStamp) > m_rtProp;

  if (isFullLength || IsTimeToProbeRenoCoexistence ())
    {
      EnterProbeRefill (tcb, 0);
      return true;
    }
  return false;
}

bool
TcpBbr::IsTimeToCruise (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  bool isFullLength = (Simulator::Now () - m_cycleStamp) > m_rtProp;

  // Reduce inflight to leave headroom
  if (tcb->m_priorInFlight > InflightWithHeadroom ())
    {
      return false;
    }
  
  return tcb->m_priorInFlight <= InFlight (tcb, 1) || isFullLength;
}

bool
TcpBbr::IsTimeToProbeRenoCoexistence ()
{
  NS_LOG_FUNCTION (this);
  uint32_t rounds = m_bwProbeMaxRounds;
  uint32_t renoGain = 1;
  uint32_t inflight = TargetInflight ();
  rounds = std::min (rounds, inflight * renoGain);
  return m_roundsSinceProbe >= rounds;
}

void
TcpBbr::UpdateCyclePhase (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);
  if (m_state != BbrMode_t::BBR_PROBE_BW || !m_isPipeFilled || AdaptUpperBounds (tcb, rs))
    {
      return;
    }

  bool isFullLength = (Simulator::Now () - m_cycleStamp) > m_rtProp;
  bool isQueuing = false;

  switch (m_cycleIndex)
  {
  // Pace at estimated bdp, if loss encountered pacing slowed down
  case BbrBwPhase::BBR_BW_PROBE_CRUISE:
    if (IsTimeToProbe (tcb))
      {
        return;
      }
    break;
  // Refill the pipe and probe further
  case BbrBwPhase::BBR_BW_PROBE_REFILL:
    if (m_roundStart)
      {
        m_bwProbeSamples = 1;
        EnterProbeUp (tcb);
      }
    break;
  // Increase pacing gain to probe for bw, exit if hit inflight_hi, loss rate too high or min rtt elapsed
  case BbrBwPhase::BBR_BW_PROBE_UP:
    if (m_prevProbeTooHigh && tcb->m_priorInFlight >= m_inflightHi)
      {
        m_isRiskyProbe = true;
      }
    else if (isFullLength && tcb->m_priorInFlight >= InFlight (tcb, m_pacingGain))
      {
        isQueuing = true;
      }
    
    if (m_isRiskyProbe || isQueuing)
      {
        m_prevProbeTooHigh = false;
        EnterProbeDown (tcb);
      }
    break;
  // Drain excess data from the queue
  case BbrBwPhase::BBR_BW_PROBE_DOWN:
    if (IsTimeToProbe (tcb))
      {
        return;
      }
    if (IsTimeToCruise (tcb))
      {
        EnterProbeCruise ();
      }
    break;
  
  default:
    NS_FATAL_ERROR ("BBR invalid cycle index");
  }
}

void
TcpBbr::EnterProbeDown (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  ResetCongestionSignals ();
  m_bwProbeUpCount = std::numeric_limits<int>::max ();
  m_roundsSinceProbe = (int) m_uv->GetValue (0, m_bwProbeRandRounds);
  m_cycleStamp = Simulator::Now ();
  m_ackPhase = BbrAckPhase::BBR_ACK_PROBE_STOPPING;
  m_nextRoundDelivered = tcb->m_delivered;
  SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_DOWN);
}

void
TcpBbr::EnterProbeUp (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);
  m_ackPhase = BbrAckPhase::BBR_ACK_PROBE_STARTING;
  m_nextRoundDelivered = tcb->m_delivered;
  m_cycleStamp = Simulator::Now ();
  SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_UP);
  RaiseInflightHiSlope (tcb);
}

void
TcpBbr::EnterProbeRefill (Ptr<TcpSocketState> tcb, uint32_t bwProbeUpRounds)
{
  NS_LOG_FUNCTION (this << tcb << bwProbeUpRounds);
  // Reset Lower Bounds
  m_inflightLo = std::numeric_limits<int>::max ();
  m_bwLo = std::numeric_limits<int>::max ();
  m_bwProbeUpRounds = bwProbeUpRounds;
  m_bwProbeUpAcks = 0;
  m_isRiskyProbe = false;
  m_ackPhase = BbrAckPhase::BBR_ACKS_REFILLING;
  m_nextRoundDelivered = tcb->m_delivered;
  SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_REFILL);
}

void
TcpBbr::EnterProbeCruise ()
{
  NS_LOG_FUNCTION (this);

  if (m_inflightLo != std::numeric_limits<int>::max ())
    {
      m_inflightLo = std::min (m_inflightLo, m_inflightHi);
    }

  SetCycleIndex (BbrBwPhase::BBR_BW_PROBE_CRUISE);
}

void
TcpBbr::BoundCwndForInflightModel (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  uint32_t cap = std::numeric_limits<int>::max ();
  if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex != BbrBwPhase::BBR_BW_PROBE_CRUISE)
    {
      cap = m_inflightHi;
    }
  else
    {
      if (m_state == BbrMode_t::BBR_PROBE_RTT || (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == BbrBwPhase::BBR_BW_PROBE_CRUISE))
        {
          cap = InflightWithHeadroom ();
        }
    }

  cap = std::min (cap, m_inflightLo);
  cap = std::max (cap, m_minPipeCwnd);
  tcb->m_cWnd = std::min<uint32_t> (cap, tcb->m_cWnd);
}

void
TcpBbr::UpdateEcn (Ptr<TcpSocketState> tcb, const struct RateSample * rs)
{
  NS_LOG_FUNCTION (this << tcb << rs);

  if (!m_enableEcn)
    {
      return;
    }

  uint32_t delivered = tcb->m_delivered - m_alphaLastDelivered;
  uint32_t deliveredEce = m_deliveredEce - m_alphaLastDeliveredEce;

  if (delivered == 0)
    {
      return;
    }

  uint32_t eceRatio = deliveredEce / delivered;
  uint32_t alpha = (1 - m_ecnGain) * m_ecnAlpha;
  alpha += m_ecnGain * eceRatio;
  m_ecnAlpha = alpha;

  m_alphaLastDelivered = tcb->m_delivered;
  m_alphaLastDeliveredEce = m_deliveredEce;

  CheckEcnTooHighStartup (tcb, rs, eceRatio);
}

void
TcpBbr::CheckEcnTooHighStartup (Ptr<TcpSocketState> tcb, const struct RateSample * rs, uint32_t ratio)
{
  if (m_isPipeFilled || !m_enableEcn)
    {
      return;
    }

  if (ratio > m_ecnThresh)
    {
      m_startupEcnRounds++;
    }
  else
    {
      m_startupEcnRounds = 0;
    }
  
  if (m_startupEcnRounds >= m_fullEcnCount)
    {
      m_isPipeFilled = true;
      m_inflightHi = InFlight(tcb, 1);
    }
}

uint32_t
TcpBbr::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << tcb << bytesInFlight);
  SaveCwnd (tcb);
  return tcb->m_initialSsThresh;
}

Ptr<TcpCongestionOps>
TcpBbr::Fork (void)
{
  return CopyObject<TcpBbr> (this);
}

} // namespace ns3
