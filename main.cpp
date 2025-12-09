/**
 * Sources:
 * - https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html
 */

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <bits/types/cookie_io_functions_t.h>
#include <cstdint>
#include <ctime>
#include <memory>
#include <random>

class Flock;

/**
 * Flocking object (boid).
 */
class Boid
{
public:
    Boid(float x, float y, float radius, sf::Color color)
        : mPos(x, y), mRadius(radius), mColor(color), mCirc(radius)
    {
    }

    /**
     * @brief	Move the boid according to its velocity.
     */
    void move() { mPos += mVel; }

    /**
     * @brief	Draws the boid to an SFML window.
     * @param	window	SFML render window.
     */
    void draw(sf::RenderWindow *window)
    {
        mCirc.setPosition(mPos - sf::Vector2f{mRadius, mRadius});
        mCirc.setFillColor(mColor);
        window->draw(mCirc);
    }

    sf::Vector2f &velocity() { return mVel; }
    const sf::Vector2f &position() const { return mPos; }
    const float &radius() const { return mRadius; }

private:
    sf::Vector2f mPos;
    sf::Vector2f mVel{0.f, 0.f};

    float mRadius;
    sf::Color mColor = sf::Color::Red;
    sf::CircleShape mCirc;
};

/**
 * Flock contains a set of boids which move in unison towards a destination.
 */
class Flock
{
public:
    /**
     * @brief	Updates velocities and positions of all boids in the flock.
     */
    void update()
    {
        static std::mt19937 rng{std::random_device{}()};
        static std::uniform_real_distribution<float> noise(-1.f, 1.f);

        for (auto &boid : mBoids)
        {
            const float &r = boid->radius();
            const sf::Vector2f &pos = boid->position();
            sf::Vector2f &vel = boid->velocity();

            sf::Vector2f separation{0.f, 0.f};
            sf::Vector2f avg_vel{0.f, 0.f};
            sf::Vector2f avg_pos{0.f, 0.f};
            int neighbourhood_size = 0;

            // Iterate over other boids
            for (auto &other_boid : mBoids)
            {
                if (boid.get() == other_boid.get())
                {
                    continue;
                }

                const float &r2 = other_boid->radius();
                const sf::Vector2f &pos2 = other_boid->position();
                sf::Vector2f &vel2 = other_boid->velocity();

                sf::Vector2f toOther = pos2 - pos;
                const float len = toOther.length();
                const float dist = len - (r + r2);
                if (dist < VISUAL_RANGE)
                {
                    separation -= toOther / (len * std::pow(2.f, dist));
                    avg_vel += vel2;
                    avg_pos += pos2;
                    neighbourhood_size++;
                }
            }

            if (neighbourhood_size > 0)
            {
                avg_vel /= static_cast<float>(neighbourhood_size);
                avg_pos /= static_cast<float>(neighbourhood_size);
            }

            sf::Vector2f toTarget = mDest - pos;
            sf::Vector2f toDest = toTarget.normalized() * MAX_SPEED;

            vel += separation * AVOID_FACTOR;                 // Separation
            vel += MATCHING_FACTOR * (avg_vel - vel);         // Alignment
            vel += CENTERING_FACTOR * (avg_pos - pos);        // Cohesion
            vel = (1.f - BIAS_VAL) * vel + BIAS_VAL * toDest; // Destination

            // Add random movements
            sf::Vector2f jitter{noise(rng) * NOISE_STRENGTH, noise(rng) * NOISE_STRENGTH};
            vel += jitter;

            // Enforce speed limit
            const float speed = vel.length();
            if (speed > MAX_SPEED)
            {
                vel *= (MAX_SPEED / (speed + 1e-2f));
            }
            else if (speed < MIN_SPEED)
            {
                vel *= (MIN_SPEED / (speed + 1e-2f));
            }

            // Update position
            boid->move();
        }
    }

    /**
     * @brief	Draw all boids to an SFML window.
     * @param	window	SFML render window.
     */
    void draw(sf::RenderWindow *window)
    {
        for (auto &boid : mBoids)
        {
            boid->draw(window);
        }
    }

    /**
     * @brief	Add a boid to the flock.
     * @param	x	        Initial X coordinate.
     * @param	y	        Initial Y coordinate.
     * @param	radius	    Radius.
     * @param	color	    Color.
     */
    void addBoid(float x, float y, float radius, sf::Color color)
    {
        mBoids.emplace_back(std::make_unique<Boid>(x, y, radius, color));
    }

    /**
     * @brief	Set the destination for all boids to move towards.
     * @param	newDest	    New destination.
     */
    void setDest(sf::Vector2f newDest) { mDest = newDest; }

    /**
     * @brief	Removes all boids.
     */
    void clear() { mBoids.clear(); }

private:
    static constexpr float AVOID_FACTOR = 0.5f;
    static constexpr float VISUAL_RANGE = 20.f;
    static constexpr float CENTERING_FACTOR = 0.0005f;
    static constexpr float MATCHING_FACTOR = 0.05f;
    static constexpr float MAX_SPEED = 6.f;
    static constexpr float MIN_SPEED = 1.f;
    static constexpr float BIAS_VAL = 0.005f;
    static constexpr float NOISE_STRENGTH = 0.1f;

    std::vector<std::unique_ptr<Boid>> mBoids;
    sf::Vector2f mDest;
};

/**
 * Runs a flock. Initializes the flock, adds boids, updates the flock based on
 * events, and draws it each frame to an SFML window.
 */
class FlockingApp
{
public:
    /**
     * @brief	Construct a new Flocking App object.
     * @param	windowWidth	    Width in pixels of the SFML window.
     * @param	windowHeight	Height in pixels of the SFML window.
     * @param	flockSize	    Number of boids in the flock.
     */
    FlockingApp(unsigned int windowWidth, unsigned int windowHeight, unsigned int flockSize)
        : mRng(time(nullptr)), mFlockSize(flockSize)
    {
        mWindow =
            sf::RenderWindow(sf::VideoMode({windowWidth, windowHeight}), "Flocking Demo (SFML)");
        mWindow.setFramerateLimit(60);
        mFlock = std::make_unique<Flock>();
        createRandomFlock(flockSize);
    }

    /**
     * @brief	Run the flocking app.
     */
    void run()
    {
        while (mWindow.isOpen())
        {
            handleEvents();

            mFlock->update();

            mWindow.clear(sf::Color::Black);
            mFlock->draw(&mWindow);
            mWindow.display();
        }
    }

private:
    sf::RenderWindow mWindow;
    std::unique_ptr<Flock> mFlock;
    mutable std::mt19937 mRng;
    unsigned int mFlockSize;

    /**
     * @brief	Handles SFML events.
     */
    void handleEvents()
    {
        while (const std::optional<sf::Event> event = mWindow.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                mWindow.close();
            }
            else if (const auto *mouseMoved = event->getIf<sf::Event::MouseMoved>())
            {
                mFlock->setDest(sf::Vector2f{mouseMoved->position});
            }
            else if (const auto *keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->scancode == sf::Keyboard::Scancode::R)
                {
                    mFlock->clear();
                    createRandomFlock(mFlockSize);
                }
            }
        }
    }

    /**
     * @brief	Adds n flock objects at a random position in the window with a random radius and
     *          random color.
     * @param	n
     */
    void createRandomFlock(unsigned int n)
    {
        for (int i = 0; i < n; i++)
        {
            auto rand = [&](uint64_t min, uint64_t max) { return mRng() % (max - min) + min; };

            int x = rand(0, mWindow.getSize().x);
            int y = rand(0, mWindow.getSize().y);
            int radius = rand(2, 8);
            int r = rand(0, 255);
            int g = rand(0, 255);
            int b = rand(0, 255);
            mFlock->addBoid(x, y, radius, sf::Color(r, g, b));
        }
    }
};

int main()
{
    FlockingApp app(1524, 1024, 300);
    app.run();

    return 0;
}
