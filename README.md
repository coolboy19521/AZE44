In this section of the documentation software solution of our strategy is discussed. Before starting to explain how our code works we would like to present our principles while creating the software:

1. **Keep it simple**: The computing power of our SBC and the accuracy of our sensors are limited. There is no point in designing an extra accurate algorithm, because it most probably would not work in practice mose of the time. Also maintaining a complex algorithmic structure is not easy and would require a lot of time, which we could more effectively spend on optimizing existing simpler algorithms or our strategy.

2. **Have a margin of error**: There is nothing such as *at STP* in the competition arena. So you can't just solely rely on robot's mechanic design and right perfect code. Software should always be able to tolerate some amount of hardware error. So should hardware, as perfection lies in balance of both.

3. **Test enough**: Sometimes we get new ideas for our strategy or overall robot behaviour. It is not appropriate to use it if it works once. Before moving on and accepting the solution, it should be tested. Many of the ideas look bright at first glance but with proper testing its faults are much more observable.
> [!NOTE]
> For every problem, there is a solution which is simple, fast, and wrong 😉.


4. **Never be afraid of trying new ideas**: Most of our ideas about the robot and strategy has changed since we started working on the actual map. Almost nothing has stayed the same 🙂. Because of that, we always think of alternative ways of doing something in parallel to implementing our already existing ideas.

5. **Stay positive**: It is really easy to get pessimistic when a solution does not work, especially when you spend a lot of time on it. In times like these we remind ourselves why we started this journey. To learn something new, experience new stuff and compete fairly.

Now that we got our principles out of the way, we can start with the actual software. Our structure for software flow is such:

**HERE WILL BE AN IMAGE**

Let's get into setting up the development environment. We have a very generic development environment for the host machine. The only programs you need to develop is `ssh` and any text editor of your choice. All of the modern machines nowadays come with these by default. To develop on the host machine we used `Command Prompt` in our Windows machine and `Terminal` in our Ubuntu machine.

Environment for the Raspberry Pi is a little bit more complex but to make it simpler we developed a solution as such:

**HERE WILL BE A FILE**

This is our Raspberry Pi image. If you flash this file into your Raspberry, you will get the exactly same environment as us. To flash this image you can use **HERE WILL BE A PROGRAM NAME**:

**HERE WILL BE ANOTHER IMAGE**

But if you want to customize installation you can just flush the Raspberry Pi from the official Raspberry Pi Imager. **HERE WILL BE A LINK TO THE FULL GUIDE** After installing the operating system you can install the ROS2 environment. **HERE WILL BE A LINK TO THE FULL GUIDE**

From now on we will talk about different section of the software as describe in figure 1.1.

Let's start with the Raspberry Pi Pico - motor driver. We have created our own API for motor and servo control which also supports motor encoders. Before developping our own motor driver we actually tried an already existing one, but it had many issues. The motor driver we used before was consistently losing connection with Raspberry Pi. It was stopping to receive any commands transmitted from the Raspberry Pi and executed the last one. We had many ideas to fix this issue:

1. **Sending large packets over USB**: Our idea for this happening was incorrect use of available transmission space inside the USB adapter. So we modified the API of our previous motor driver. This hypothesis was actually correct to some degree: before making the modification our robot lost connection at approximately `5th minute` of working, but after making the modification we actually quadrupled this number with `20 minutes` lifespan. But robot was still losing connection 😢.

2. **Sending too much motor commands**: This was another hypothesis of ours, and to try it out we lowered the frequency of our Raspberry Pi solution. This actually helped a lot as now the lifespan of the motor driver was up to `40 minutes`. This was some improvement, but motor drivers are designed to work as much as they are powered, not periodically 😅.

3. **Losing bytes when receiving the packets**: We had really good faith in this idea. Because we thought the issue was most probably this. But when we tried adding addition checksums and start flags to the packet for security, nothing changed at all. We did not observe any improvement in lifespan of the motor driver.

After having these issues we decided it was not worth risking a faulty motor driver. When we made this decision there was only a week left before the national qualifications so we were too late to buy a new motor driver. That's why we decided to make our own using Raspberry Pi Pico. Pico receives the commands from Raspberry Pi and sends them to the internat motor driver. When designing the work flow of our API we relied on our previous hypothesis of why a motor drive could fail. The new API we have created has never failed since we have created it and we hope it will stay so. Let's look at some cruicial parts and explain them on code:

**HERE WILL BE A CODE AND COMMENTS EXPLAINING HOW THIS WORKS**

We also have an API file on Raspberry Pi side to communicate with Raspberry Pi Pico:

**HERE WILL BE A CODE AND COMMENTS EXPLAINING HOW THIS WORKS**

> [!NOTE]
> Our motor driver API support motor commands at a frequency up to `50 Hz`, and we are using it at `20 Hz`. The limit for our usage is bound by our strategy and the overall logic for the solution.
