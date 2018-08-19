#ifndef _RBK_CORE_ACTION_H_
#define _RBK_CORE_ACTION_H_

#include <robokit/config.h>
#include <robokit/core/logger.h>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <boost/asio.hpp>
#include "message_task.pb.h"

namespace rbk {
    namespace core {
		enum RBKStatus{
			NONE	= 0,
			WAITING		= 1,
			RUNNING		= 2,
			SUSPENDED	= 3,
			FINISHED	= 4,
			FAILED		= 5,
			CANCELED	= 6,
			OVERTIME	= 7
		};

		using rbk::protocol::Message_Action;
		//class Message_Task;
        class RBK_API Action {
			public:
				Action(std::string name);
				void init(std::function<void(Message_Action*)> func);
				void start(Message_Action* msg);
				void cancel();
				void setCancelFunc(std::function<void()> func);
				void suspend();
				void setSuspendFunc(std::function<void()> func);
				void resume();
				void setResumeFunc(std::function<void()> func);
				void finish();
				void setFinishFunc(std::function<void()> func);
				void fail();
				void setFailFunc(std::function<void()> func);
				void overtime(const boost::system::error_code& err);
				void setOverTimeFunc(std::function<void()> func);
				bool externalOverId(int&);
				void startTimer();
				RBKStatus status();
				void sleepFunc(double);
				void getResult(rbk::protocol::Message_ActionResult&);
				void setResult(rbk::protocol::Message_ActionResult);
				bool needResult();
			private:
				void clearOperation();
				std::atomic<RBKStatus> m_status;
				std::string m_name;
				bool m_initialized;
				std::function<void(Message_Action*)> m_run_func;
				std::function<void()> m_cancel_func;
				std::function<void()> m_suspend_func;
				std::function<void()> m_resume_func;
				std::function<void()> m_finish_func;
				std::function<void()> m_fail_func;
				std::function<void()> m_overtime_func;
				uint64_t m_start_time;
				uint64_t m_interval;
				Message_Action* m_msg;
				rbk::protocol::Message_ActionResult m_res;
				std::mutex m_res_mutex;
				std::mutex m_msg_mutex;
				boost::asio::io_service m_io;
				boost::asio::deadline_timer m_timer;
        };

		class RBK_API ActionManager{
			private:
				struct object_creator {
					object_creator() { ActionManager::Instance(); }
					inline void do_nothing() const { }
				};

				static object_creator create_object;
				ActionManager(){}
				~ActionManager(){}
			public:
				static ActionManager* Instance();

				Action* createAction(std::string action_name, std::function<void(rbk::protocol::Message_Action*)> func);
				Action* startAction(std::string plugin_name, std::string action_name, Message_Action* msg);
				void updateExternalOverId(int id, bool value);
				bool status(std::string plugin_name, std::string action_name, RBKStatus& s);
				bool finishAction(std::string plugin_name, std::string action_name, rbk::protocol::Message_ActionResult* res = NULL);

				//template<typename T>
				//bool finishAction(std::string plugin_name, std::string action_name, T& res){
				//	std::string final_name = plugin_name; 
				//	final_name.append("_").append(action_name);
				//	std::map<std::string, Action*>::const_iterator iter = m_action_map.find(final_name);
				//	if (m_action_map.end() != iter) {
				//		iter->second->setResult(res);
				//		iter->second->finish();
				//		return true;
				//	} else {
				//		LogError("Can not find action : " << final_name);
				//		return false;
				//	}
				//}

				bool cancelAction(std::string plugin_name, std::string action_name);
				bool failAction(std::string plugin_name, std::string action_name);
				bool suspendAction(std::string plugin_name, std::string action_name);
				bool resumeAction(std::string plugin_name, std::string action_name);
				Action* getAction(std::string plugin_name, std::string action_name);
				bool setActionResult(std::string plugin_name, std::string action_name, rbk::protocol::Message_ActionResult*);
			private:
				std::map<std::string, Action*> m_action_map;
				std::map<int, bool> m_external_over_id_map;
				std::mutex m_external_over_id_mutex;
		};
    } // namespace core
} // namespace rbk

#endif // ~_RBK_CORE_ACTION_H_
