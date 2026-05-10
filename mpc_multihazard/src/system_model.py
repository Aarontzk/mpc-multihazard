# ASUMSI: Full State Feedback
# x(k) dibaca langsung dari model simulasi
# Tidak merepresentasikan sistem sensor fisik
# Untuk implementasi hardware: tambahkan Kalman Filter

import numpy as np
from scipy.signal import cont2discrete


def build_continuous_matrices(M, C, K, L_u, L_d):
    """
    Bangun matriks sistem kontinu dari parameter struktural.

    Args:
        M   : mass matrix (num_dof x num_dof)
        C   : damping matrix (num_dof x num_dof)
        K   : stiffness matrix (num_dof x num_dof)
        L_u : input location matrix — pemetaan aktuator (MLFS, FAHFS)
        L_d : disturbance location matrix — pemetaan gangguan (gempa, fluida)

    Returns:
        A_cont (2n x 2n), B_cont (2n x m), E_cont (2n x d)
    """
    num_dof = M.shape[0]
    Z = np.zeros((num_dof, num_dof))
    I = np.eye(num_dof)
    M_inv = np.linalg.inv(M)

    # Matriks A
    A_top = np.hstack((Z, I))
    A_bottom = np.hstack((-np.dot(M_inv, K), -np.dot(M_inv, C)))
    A_cont = np.vstack((A_top, A_bottom))

    # Matriks B (pemetaan u: MLFS, FAHFS)
    B_top = np.zeros((num_dof, L_u.shape[1]))
    B_bottom = np.dot(M_inv, L_u)
    B_cont = np.vstack((B_top, B_bottom))

    # Matriks E (pemetaan d: gempa, fluida)
    E_top = np.zeros((num_dof, L_d.shape[1]))
    E_bottom = np.dot(M_inv, L_d)
    E_cont = np.vstack((E_top, E_bottom))

    return A_cont, B_cont, E_cont


def discretize_system(A_cont, B_cont, E_cont, Ts):
    """
    Diskritisasi sistem kontinu ke diskrit dengan ZOH.
    B dan E diaugmentasi menjadi satu kolom input agar satu kali panggil cont2discrete.

    Args:
        A_cont, B_cont, E_cont : matriks sistem kontinu
        Ts                     : sampling time (detik)

    Returns:
        A_disc, B_disc, E_disc
    """
    num_states = A_cont.shape[0]
    num_u = B_cont.shape[1]

    # Augmentasi input: [u; d] sekaligus
    B_aug_cont = np.hstack((B_cont, E_cont))

    # ASUMSI: Full State Feedback → C = I, D = 0
    C_cont = np.eye(num_states)
    D_aug_cont = np.zeros((num_states, num_u + E_cont.shape[1]))

    sys_disc = cont2discrete((A_cont, B_aug_cont, C_cont, D_aug_cont), Ts, method='zoh')

    A_disc = sys_disc[0]
    B_aug_disc = sys_disc[1]

    # Ekstraksi kembali
    B_disc = B_aug_disc[:, :num_u]
    E_disc = B_aug_disc[:, num_u:]

    return A_disc, B_disc, E_disc
