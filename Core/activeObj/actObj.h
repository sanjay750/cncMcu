/*
 * actObj.h
 *
 *  Created on: 23-Aug-2022
 *      Author: sanjay
 */

#ifndef ACTIVEOBJ_ACTOBJ_H_
#define ACTIVEOBJ_ACTOBJ_H_

enum QStateRet {
    /* unhandled and need to "bubble up" */
    Q_RET_SUPER,     /*!< event passed to superstate to handle */
    Q_RET_SUPER_SUB, /*!< event passed to submachine superstate */
    Q_RET_UNHANDLED, /*!< event unhandled due to a guard */

    /* handled and do not need to "bubble up" */
    Q_RET_HANDLED,   /*!< event handled (internal transition) */
    Q_RET_IGNORED,   /*!< event silently ignored (bubbled up to top) */

    /* entry/exit */
    Q_RET_ENTRY,     /*!< state entry action executed */
    Q_RET_EXIT,      /*!< state exit  action executed */

    /* no side effects */
    Q_RET_NULL,      /*!< return value without any effect */

    /* transitions need to execute transition-action table in ::QMsm */
    Q_RET_TRAN,      /*!< regular transition */
    Q_RET_TRAN_INIT, /*!< initial transition in a state or submachine */
    Q_RET_TRAN_EP,   /*!< entry-point transition into a submachine */

    /* transitions that additionally clobber me->state */
    Q_RET_TRAN_HIST, /*!< transition to history of a given state */
    Q_RET_TRAN_XP    /*!< exit-point transition out of a submachine */
};

typedef enum QStateRet QState;
typedef QState (* QStateHandler )(void * const me, QEvt const * const e);
typedef QState (* QActionHandler )(void * const me);

typedef struct QMState {
    struct QMState const *superstate; /*!< superstate of this state */
    QStateHandler const stateHandler; /*!< state handler function */
    QActionHandler const entryAction; /*!< entry action handler function */
    QActionHandler const exitAction;  /*!< exit action handler function */
    QActionHandler const initAction;  /*!< init action handler function */
} QMState;

enum QReservedSig {
    Q_ENTRY_SIG = 1, /*!< signal for coding entry actions */
    Q_EXIT_SIG,      /*!< signal for coding exit actions */
    Q_INIT_SIG,      /*!< signal for coding initial transitions */
    Q_USER_SIG       /*!< offset for the user signals (QP Application) */
};



#endif /* ACTIVEOBJ_ACTOBJ_H_ */
