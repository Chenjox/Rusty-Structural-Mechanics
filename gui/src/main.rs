use iced::pure::widget::Text;
use iced::pure::{container, text};
use iced::pure::{Application, Element};
use iced::{executor, Command, Length, Settings};

pub fn main() -> iced::Result {
    Start::run(Settings::default())
}

struct Start;

impl Application for Start {
    type Executor = executor::Default;
    type Message = ();
    type Flags = ();

    fn new(_flags: ()) -> (Start, Command<Self::Message>) {
        (Start, Command::none())
    }

    fn title(&self) -> String {
        String::from("Rusty Structural Mechanics")
    }

    fn update(&mut self, _message: Self::Message) -> Command<Self::Message> {
        Command::none()
    }

    fn view(&self) -> Element<Self::Message> {
        container(text("Test"))
            .center_x()
            .center_y()
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}
