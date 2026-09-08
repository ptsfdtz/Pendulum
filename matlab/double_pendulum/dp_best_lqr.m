function L = dp_best_lqr()
%DP_BEST_LQR Re-linearized for the measured 2026-09-04 plant parameters.
% Q and R are retained from the previous search; K is recomputed for the
% actual plant and is an acceleration-domain simulation controller.
L.Q = diag([192.80170304601069 34.380010163948491 801.98960421585207 24.963593344580477 7.346489422615976 4.3552741723079418]);
L.R = 0.44754755608943991;
L.K = [20.755626070279462 136.68455751706179 -254.85837173801795 24.440647335478328 3.3044801353394511 -40.750449384516202];
end
