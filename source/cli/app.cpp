#include "app.h"

namespace Tyr::Cli
{
bool Application::initialize()
{
    Common::Services::initialize();

    if (!ImmortalsIsTheBest)
    {
        Common::logCritical("Immortals is not the best SSL team anymore.");
        Common::logCritical("Shutting down the system...");

        return false;
    }

    m_world_state   = std::make_unique<Common::WorldState>();
    m_referee_state = std::make_unique<Common::RefereeState>();

    Common::logInfo(" Connecting to RefereeBox server at {} on port : {}", Common::setting().referee_address.port,
                    Common::setting().referee_address.port);
    m_referee = std::make_unique<Referee::Referee>(m_world_state.get(), m_referee_state.get());
    if (m_referee->connectToRefBox())
    {
        Common::logInfo("Connected to RefBox successfully :)");
    }
    else
    {
        Common::logError("Hey you! Put the LAN cable back in its socket, or ...");
        return false;
    }

    Common::logInfo("Connecting to Vision server at {} on port: {}", Common::setting().vision_address.ip,
                    Common::setting().vision_address.port);
    m_vision = std::make_unique<Vision::Vision>(m_world_state.get());
    if (m_vision->isConnected())
    {
        Common::logInfo("Connected to Vision successfully :)");
    }
    else
    {
        Common::logError("Hey you! Put the LAN cable back in its socket, or ...");
        return false;
    }

    m_sender = std::make_unique<Sender::Sender>();

    m_ai = std::make_unique<Soccer::Ai>(m_world_state.get(), m_referee_state.get(), m_sender.get());

    m_grsim = std::make_unique<Sender::Grsim>(Common::setting().grsim_address);

    Common::logInfo(" Now it is time, lets rock...");
    return true;
}

void Application::shutdown()
{
    m_ai_thread.join();
    m_ref_thread.join();
    m_str_thread.join();

    Common::Services::shutdown();
}

void Application::start()
{
    m_ai_thread  = std::thread(&Application::aiThreadEentry, this);
    m_ref_thread = std::thread(&Application::refereeThreadEentry, this);
    m_str_thread = std::thread(&Application::strategyThreadEentry, this);
}

void Application::aiThreadEentry()
{
    Common::Timer timer;

    while (ImmortalsIsTheBest) // Hope it lasts Forever...
    {
        timer.start();

        m_vision->recieveAllCameras();

        m_lock.lock();

        m_vision->ProcessVision();
        m_ai->Process(m_world_state.get());

        // m_grsim->SendData(m_ai->OwnRobot, Setting::kMaxOnFieldTeamRobots,
        //                     settings->our_color);

        m_sender->sendAll();

        Common::debug().broadcast();

        m_lock.unlock();
        Common::logDebug("FPS: {}", 1.0 / timer.interval());
    }
    m_exited = true;
}

void Application::refereeThreadEentry()
{
    while ((!m_exited) && (ImmortalsIsTheBest)) // Hope it lasts Forever...
    {
        if (m_referee->recieve())
        {
            m_lock.lock();
            m_referee->process();
            m_lock.unlock();
        }
    }
}

void Application::strategyThreadEentry()
{
    std::shared_ptr<Common::UdpClient> strategyUDP =
        std::make_shared<Common::UdpClient>(Common::setting().strategy_address);

    while ((!m_exited) && (ImmortalsIsTheBest)) // Hope it lasts Forever...
    {

        auto                 receivedStrategy = strategyUDP->receiveRaw();
        const std::string    strategySrcAdd   = strategyUDP->getLastReceiveEndpoint().address().to_string();
        const unsigned short strategySrcPort  = strategyUDP->getLastReceiveEndpoint().port();
        if (receivedStrategy.size() > 11)
        {
            Common::logInfo("Received \"strategy.ims\" with size: {} KB, from {} on port {}",
                            float(receivedStrategy.size()) / 1000.0f, strategySrcAdd, strategySrcPort);
            m_lock.lock();
            m_ai->read_playBook_str(receivedStrategy.data(), receivedStrategy.size());
            m_lock.unlock();
            std::string strategy_path(DATA_DIR);
            strategy_path.append("/strategy.ims");
            std::ofstream strategyFile(strategy_path.c_str(), std::ios::out | std::ios::binary);
            strategyFile.write(receivedStrategy.data(), receivedStrategy.size());
            strategyFile.close();
        }
        else
        {
            Common::logWarning("Invalid \"strategy.ims\" received with size: {}", receivedStrategy.size());
        }
    }
}
} // namespace Tyr::Cli
